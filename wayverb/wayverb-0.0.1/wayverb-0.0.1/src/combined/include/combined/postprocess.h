#pragma once

#include "raytracer/postprocess.h"

#include "waveguide/canonical.h"
#include "waveguide/config.h"
#include "waveguide/dispersion_compensation.h"
#include "waveguide/postprocess.h"

#include "core/sum_ranges.h"

#include "audio_file/audio_file.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <complex>
#include <cstdio>
#include <numeric>
#include <type_traits>
#include <vector>

namespace wayverb {
namespace combined {

template <typename Histogram>
struct combined_results final {
    raytracer::simulation_results<Histogram> raytracer;
    util::aligned::vector<waveguide::bandpass_band> waveguide;
};

template <typename Histogram>
auto make_combined_results(
        raytracer::simulation_results<Histogram> raytracer,
        util::aligned::vector<waveguide::bandpass_band> waveguide) {
    return combined_results<Histogram>{std::move(raytracer),
                                       std::move(waveguide)};
}

////////////////////////////////////////////////////////////////////////////////
//  Direct sound
////////////////////////////////////////////////////////////////////////////////
//
//  The direct sound now bypasses the multiband filterbank entirely.
//  raytracer::postprocess() places it as a clean broadband sinc spike
//  directly into the output signal and reports the amplitude/distance
//  via raytracer::postprocessed_result.  No injection hack needed here.

//  Strip leading silence from the IR.
//  Finds the first sample whose magnitude exceeds a fraction of the peak,
//  then trims everything before it (with a small safety margin).
//  This is robust to alignment shifts and onset delays that the old
//  distance-based trim couldn't account for.
template <typename Vec>
void trim_leading_silence(Vec& v, double sample_rate) {
    if (v.size() < 64) return;

    float peak = 0.0f;
    for (const auto& s : v) peak = std::max(peak, std::abs(s));
    if (peak < 1e-10f) return;

    //  Two-pass onset detection:
    //  1. Find the main direct-sound peak in the first half of the signal.
    //  2. Search backwards from that peak for -40 dB (1% of peak).
    //  This skips over low-level FFT pre-ring that defeats simple thresholds.
    const auto half = v.size() / 2;
    size_t peak_idx = 0;
    for (size_t i = 0; i < half; ++i) {
        if (std::abs(v[i]) > std::abs(v[peak_idx])) peak_idx = i;
    }

    const float thresh = peak * 0.01f;  // -40 dB
    size_t onset = peak_idx;
    for (size_t i = peak_idx; i > 0; --i) {
        if (std::abs(v[i]) < thresh) {
            onset = i;
            break;
        }
    }

    //  Keep 32 samples before onset for pre-ring safety margin.
    const size_t trim = onset > 32 ? onset - 32 : 0;
    if (trim == 0) return;

    v.erase(v.begin(), v.begin() + trim);
    fprintf(stderr,
            "[trim_leading_silence] removed %zu samples (%.1fms)\n",
            trim, 1000.0 * trim / sample_rate);
    fflush(stderr);
}

////////////////////////////////////////////////////////////////////////////////
//  IR quality helpers
////////////////////////////////////////////////////////////////////////////////

//  Compute mixing time from room volume (Kuttruff formula).
//  Must match the value used in stochastic synthesis.
inline double compute_mixing_time(double room_volume, double speed_of_sound) {
    const auto t_mix = std::sqrt(
            room_volume / (4.0 * M_PI * std::pow(speed_of_sound, 3.0)));
    return std::max(0.005, std::min(t_mix, 2.0));
}

//  Enforce monotonic energy decay after the mixing time.
//  Only acts on the late/diffuse portion of the IR — early reflections
//  (before mixing time) are left untouched since they naturally have
//  variable amplitude.
template <typename Vec>
void enforce_decay(Vec& v, double sample_rate, double mixing_time) {
    if (v.size() < 2048) return;

    //  Use 1024-sample windows (23ms at 44.1kHz) to detect shorter
    //  unphysical bumps while still averaging over individual reflections.
    constexpr size_t win = 1024;
    const size_t nw = v.size() / win;
    if (nw < 2) return;

    //  Start enforcement after the mixing time (early reflections are exempt).
    const size_t start_w = static_cast<size_t>(
            mixing_time * sample_rate / static_cast<double>(win));

    //  Compute RMS per window.
    std::vector<float> wrms(nw, 0.0f);
    for (size_t w = 0; w < nw; ++w) {
        float sum = 0.0f;
        for (size_t i = w * win; i < (w + 1) * win; ++i) {
            sum += v[i] * v[i];
        }
        wrms[w] = std::sqrt(sum / static_cast<float>(win));
    }

    //  Find peak RMS in [start_w, start_w + search_range].
    const size_t search_end = std::min(nw, start_w + nw / 4);
    size_t peak_w = start_w;
    float peak_v = 0.0f;
    for (size_t w = start_w; w < search_end; ++w) {
        if (wrms[w] > peak_v) {
            peak_v = wrms[w];
            peak_w = w;
        }
    }

    if (peak_v < 1e-10f) return;  //  Signal is silent, nothing to do.

    //  Enforce monotonic decay after peak.
    float ceiling = peak_v;
    for (size_t w = peak_w + 1; w < nw; ++w) {
        if (wrms[w] > ceiling && ceiling > 0.0f) {
            const float scale = ceiling / wrms[w];
            for (size_t i = w * win; i < (w + 1) * win && i < v.size(); ++i) {
                v[i] *= scale;
            }
        } else {
            ceiling = wrms[w];
        }
    }
}

//  Distance-based IR normalization.
//  Scales the IR so the direct sound peak = 1/distance (inverse distance law).
//  At 1m: peak = 1.0.  At 3m: peak = 0.33.  At 0.5m: peak = 2.0.
//  This preserves the physical room gain and distance cue.
//  Values > 1.0 are fine in 32-bit float; the global ceiling limiter in
//  threaded_engine prevents clipping for integer output formats.
//
//  If injected_direct_amp > 0, use it as the reference (we know exactly what
//  the direct sound peak should be because we injected it).  This avoids the
//  old failure mode where the stochastic tail's random peaks were mistaken
//  for the reference.
template <typename Vec>
void normalize_distance(Vec& v, double sample_rate, float distance,
                        float injected_direct_amp = 0.0f) {
    if (v.empty() || distance < 0.01f) return;

    const float target = 1.0f / std::max(distance, 0.1f);

    float ref = 0.0f;

    if (injected_direct_amp > 1e-6f) {
        //  We injected the direct sound at a known amplitude — use it.
        ref = injected_direct_amp;
    } else {
        //  Fallback: search for the direct sound peak in the first 100ms.
        const size_t search_len = std::min(v.size(),
                static_cast<size_t>(0.1 * sample_rate));
        float direct_peak = 0.0f;
        for (size_t i = 0; i < search_len; ++i) {
            direct_peak = std::max(direct_peak, std::abs(v[i]));
        }

        float global_peak = 0.0f;
        for (const auto& s : v) {
            global_peak = std::max(global_peak, std::abs(s));
        }

        ref = (direct_peak > global_peak * 0.05f)
                ? direct_peak : global_peak;
    }

    if (ref > 1e-6f) {
        const auto scale = target / ref;
        for (auto& s : v) {
            s *= scale;
        }
        fprintf(stderr,
                "[normalize_distance] d=%.2fm target=%.3f ref=%.6f "
                "scale=%.3f (injected=%s)\n",
                distance, target, ref, scale,
                injected_direct_amp > 1e-6f ? "yes" : "no");
        fflush(stderr);
    }
}

//  DC-block a signal with a high-pass filter.
template <typename Vec>
void dc_block(Vec& v, double sample_rate, double cutoff_hz = 20.0,
              double /*width*/ = 0.3) {
    if (v.empty()) return;
    //  Use a causal 2nd-order Butterworth high-pass instead of the old FFT
    //  filter.  The FFT approach with 4x zero-padding introduced broadband
    //  pre-ring before the direct sound spike — a non-causal artefact that
    //  contaminated the first 0-50 ms of the IR.  A causal biquad has zero
    //  pre-ring while still removing DC/sub-bass rumble.
    const double w0 = 2.0 * M_PI * cutoff_hz / sample_rate;
    const double alpha = std::sin(w0) / (2.0 * std::sqrt(2.0));  // Q = sqrt(2)/2
    const double cos_w0 = std::cos(w0);
    const double a0 = 1.0 + alpha;
    const double b0 = ((1.0 + cos_w0) / 2.0) / a0;
    const double b1 = (-(1.0 + cos_w0))       / a0;
    const double b2 = ((1.0 + cos_w0) / 2.0) / a0;
    const double a1 = (-2.0 * cos_w0)         / a0;
    const double a2 = (1.0 - alpha)           / a0;

    //  Forward pass.
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    for (auto& s : v) {
        double x0 = s;
        double y0 = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2;
        x2 = x1; x1 = x0;
        y2 = y1; y1 = y0;
        s = static_cast<float>(y0);
    }
    //  Reverse pass for zero-phase (linear phase) without pre-ring.
    x1 = x2 = y1 = y2 = 0;
    for (auto it = v.rbegin(); it != v.rend(); ++it) {
        double x0 = *it;
        double y0 = b0*x0 + b1*x1 + b2*x2 - a1*y1 - a2*y2;
        x2 = x1; x1 = x0;
        y2 = y1; y1 = y0;
        *it = static_cast<float>(y0);
    }
}

////////////////////////////////////////////////////////////////////////////////
//  Analytical tail extension (replaces FDN).
//  Extends the IR with per-band exponentially-decaying noise, where each
//  band decays at its own Eyring RT60 rate.  This is physically motivated
//  (Sabine/Eyring theory) and produces frequency-dependent decay that
//  matches the room's absorption characteristics.
////////////////////////////////////////////////////////////////////////////////

template <typename Vec>
void extend_with_analytical_tail(Vec& signal, double sr, double room_volume,
                                  double speed_of_sound) {
    if (signal.empty() || room_volume <= 0.0) return;

    //  Measure the signal's current duration.
    const double sig_dur = static_cast<double>(signal.size()) / sr;

    //  Estimate broadband RT60 from Sabine formula (rough).
    //  We use the signal's own decay to calibrate.
    //  Find the RMS in the last 10% of the signal to estimate tail level.
    const auto tail_start = signal.size() * 9 / 10;
    double tail_rms = 0.0;
    for (size_t i = tail_start; i < signal.size(); ++i) {
        tail_rms += static_cast<double>(signal[i]) * signal[i];
    }
    tail_rms = std::sqrt(tail_rms / std::max(signal.size() - tail_start, size_t{1}));

    if (tail_rms < 1e-10) return;  //  Signal already silent at tail.

    //  Estimate the overall decay rate from the signal.
    //  Find RMS at 50% and 90% points, compute effective RT60.
    const auto mid_start = signal.size() / 2;
    const auto mid_end = signal.size() * 6 / 10;
    double mid_rms = 0.0;
    for (size_t i = mid_start; i < mid_end; ++i) {
        mid_rms += static_cast<double>(signal[i]) * signal[i];
    }
    mid_rms = std::sqrt(mid_rms / std::max(mid_end - mid_start, size_t{1}));

    double est_rt60 = 2.0;  //  Default fallback.
    if (mid_rms > tail_rms * 1.1) {
        //  Decay rate from two-point measurement.
        const double t_mid = static_cast<double>(mid_start + mid_end) / (2.0 * sr);
        const double t_tail = static_cast<double>(tail_start + signal.size()) / (2.0 * sr);
        const double decay_db = 20.0 * std::log10(tail_rms / mid_rms);
        if (decay_db < -0.5) {
            est_rt60 = -60.0 * (t_tail - t_mid) / decay_db;
            est_rt60 = std::clamp(est_rt60, 0.1, 20.0);
        }
    }

    //  Extension duration: enough to reach -60 dB from current tail level.
    const double current_db = 20.0 * std::log10(tail_rms + 1e-30);
    const double remaining_db = std::max(0.0, 60.0 + current_db);
    const double ext_time = (remaining_db / 60.0) * est_rt60;
    if (ext_time < 0.01) return;

    const auto ext_samples = static_cast<size_t>(ext_time * sr);
    if (ext_samples == 0) return;

    //  Generate exponentially-decaying noise tail.
    std::default_random_engine rng{std::random_device{}()};
    std::normal_distribution<float> noise_dist(0.0f, 1.0f);

    const auto old_size = signal.size();
    signal.resize(old_size + ext_samples, 0.0f);

    //  Decay constant: amplitude envelope exp(-6.91 * t / RT60).
    const double decay_const = 6.91 / est_rt60;

    //  100ms raised-cosine cross-fade at junction.
    const auto xfade_len = std::min(ext_samples,
                                     static_cast<size_t>(0.1 * sr));

    for (size_t i = 0; i < ext_samples; ++i) {
        const double t = static_cast<double>(i) / sr;
        const auto amp = static_cast<float>(
                tail_rms * std::exp(-decay_const * t));
        float sample = amp * noise_dist(rng);

        //  Cross-fade: blend with existing signal tail.
        if (i < xfade_len) {
            const auto w = static_cast<float>(i) / static_cast<float>(xfade_len);
            const auto old_idx = old_size - xfade_len + i;
            if (old_idx < old_size) {
                signal[old_size - xfade_len + i] =
                        signal[old_idx] * (1.0f - w) + sample * w;
                continue;
            }
        }
        signal[old_size + i - xfade_len] = sample;
    }

    fprintf(stderr,
            "[analytical_tail] extended by %.2fs (RT60≈%.2fs tail_rms=%.6f)\n",
            ext_time, est_rt60, tail_rms);
    fflush(stderr);
}

//  Temporal alignment via cross-correlation.
//  Finds the optimal lag between two signals by maximizing their
//  cross-correlation in the early portion (first 8192 samples).
//  More robust than threshold-based onset detection, which fails
//  when the direct sound is weak or occluded.
template <typename VecA, typename VecB>
void align_peaks(VecA& a, VecB& b) {
    if (a.empty() || b.empty()) return;

    //  Use only the early portion for correlation (direct sound region).
    const auto corr_len = std::min({a.size(), b.size(), size_t{8192}});
    const auto max_lag = std::min(corr_len / 2, size_t{2048});

    //  Find the lag that maximizes cross-correlation.
    float best_corr = -1e30f;
    int best_lag = 0;  //  positive = a leads b, negative = b leads a

    for (int lag = -static_cast<int>(max_lag);
         lag <= static_cast<int>(max_lag); ++lag) {
        float sum = 0.0f;
        size_t count = 0;
        for (size_t i = 0; i < corr_len; ++i) {
            const auto j = static_cast<int>(i) + lag;
            if (j >= 0 && j < static_cast<int>(corr_len)) {
                sum += a[i] * b[j];
                ++count;
            }
        }
        if (count > 0) sum /= static_cast<float>(count);
        if (sum > best_corr) {
            best_corr = sum;
            best_lag = lag;
        }
    }

    if (best_lag == 0) return;

    const auto shift = [](auto& v, size_t amount) {
        if (amount == 0 || amount >= v.size()) return;
        std::copy_backward(v.begin(), v.end() - amount, v.end());
        std::fill(v.begin(), v.begin() + amount, 0.0f);
    };

    if (best_lag > 0) {
        //  a leads b — shift b forward (or a backward).
        shift(b, static_cast<size_t>(best_lag));
        fprintf(stderr, "[align_peaks] cross-corr: shifted raytracer "
                "forward by %d samples\n", best_lag);
    } else {
        shift(a, static_cast<size_t>(-best_lag));
        fprintf(stderr, "[align_peaks] cross-corr: shifted waveguide "
                "forward by %d samples\n", -best_lag);
    }
    fflush(stderr);
}

////////////////////////////////////////////////////////////////////////////////

/// Phase-coherent crossover filter.
template <typename LoIt, typename HiIt>
auto crossover_filter(LoIt b_lo,
                      LoIt e_lo,
                      HiIt b_hi,
                      HiIt e_hi,
                      double cutoff,
                      double width) {
    const auto lo_len = std::distance(b_lo, e_lo);
    const auto hi_len = std::distance(b_hi, e_hi);
    const auto max_len = std::max(lo_len, hi_len);

    auto fft_bins = frequency_domain::best_fft_length(max_len) << 3;
    fprintf(stderr,
            "[crossover_filter] lo_len=%td hi_len=%td fft_bins=%zu "
            "cutoff=%.4f width=%.4f (phase-coherent)\n",
            lo_len, hi_len, fft_bins, cutoff, width);
    fflush(stderr);

    std::vector<float> lo_padded(max_len, 0.0f);
    std::copy(b_lo, e_lo, lo_padded.begin());
    std::vector<float> hi_padded(max_len, 0.0f);
    std::copy(b_hi, e_hi, hi_padded.begin());

    frequency_domain::filter filt{fft_bins};
    const auto cbuf_size = fft_bins / 2 + 1;

    std::vector<std::complex<float>> lo_spectrum(cbuf_size);
    size_t bin_idx = 0;
    filt.run(lo_padded.begin(), lo_padded.end(), lo_padded.begin(),
             [&](auto cplx, auto /*freq*/) {
                 lo_spectrum[bin_idx++] = cplx;
                 return std::complex<float>{0.0f, 0.0f};
             });

    constexpr auto l = 0;
    bin_idx = 0;
    util::aligned::vector<float> result(max_len, 0.0f);
    filt.run(hi_padded.begin(), hi_padded.end(), result.begin(),
             [&](auto cplx_hi, auto freq) {
                 const auto cplx_lo = lo_spectrum[bin_idx++];

                 const auto lo_w = static_cast<float>(
                         frequency_domain::compute_lopass_magnitude(
                                 freq, cutoff, width, l));
                 const auto hi_w = static_cast<float>(
                         frequency_domain::compute_hipass_magnitude(
                                 freq, cutoff, width, l));

                 const auto mag_lo = std::abs(cplx_lo);
                 const auto mag_hi = std::abs(cplx_hi);
                 const auto blended_mag = lo_w * mag_lo + hi_w * mag_hi;

                 const auto phase_lo = std::arg(cplx_lo);
                 const auto phase_hi = std::arg(cplx_hi);

                 // Use the phase of whichever signal dominates at this frequency.
                 // Smooth transition avoids phase jumps.
                 const auto blended_phase = (lo_w > hi_w) ? phase_lo : phase_hi;

                 return std::polar(blended_mag, blended_phase);
             });

    return result;
}

////////////////////////////////////////////////////////////////////////////////

struct max_frequency_functor final {
    template <typename T>
    auto operator()(T&& t) const {
        return t.valid_hz.get_max();
    }
};

template <typename Histogram, typename Method>
auto postprocess(const combined_results<Histogram>& input,
                 const Method& method,
                 const glm::vec3& source_position,
                 const glm::vec3& receiver_position,
                 double room_volume,
                 const core::environment& environment,
                 double output_sample_rate) {
    fprintf(stderr, "[combined::postprocess] start: waveguide bands=%zu room_vol=%.1f sr=%.0f\n",
            input.waveguide.size(), room_volume, output_sample_rate);
    fflush(stderr);

    const auto mixing_time = compute_mixing_time(
            room_volume, environment.speed_of_sound);
    fprintf(stderr, "[combined::postprocess] mixing_time=%.3fs\n", mixing_time);
    fflush(stderr);

    //  Individual processing.
    fprintf(stderr, "[combined::postprocess] waveguide postprocess...\n"); fflush(stderr);
    auto waveguide_processed =
            waveguide::postprocess(input.waveguide,
                                   method,
                                   environment.acoustic_impedance,
                                   output_sample_rate);
    fprintf(stderr, "[combined::postprocess] waveguide done, len=%zu\n",
            waveguide_processed.size()); fflush(stderr);

    //  Dispersion compensation: warp the waveguide spectrum to correct
    //  for the IWB stencil's residual frequency-dependent phase velocity error.
    //  The IWB 19-point scheme has much less dispersion than the old 7-point,
    //  but still benefits from post-hoc correction above ~80% Nyquist.
    if (!input.waveguide.empty()) {
        const auto wg_sr = input.waveguide.front().band.sample_rate;
        waveguide::compensate_dispersion(
                waveguide_processed, wg_sr, output_sample_rate);
    }

    fprintf(stderr, "[combined::postprocess] raytracer postprocess...\n"); fflush(stderr);
    auto rt_result = raytracer::postprocess(input.raytracer,
                                            method,
                                            receiver_position,
                                            room_volume,
                                            environment,
                                            output_sample_rate);
    auto raytracer_processed = std::move(rt_result.signal);
    const auto direct_amp = rt_result.direct_amplitude;
    const auto direct_dist = rt_result.direct_distance;
    fprintf(stderr, "[combined::postprocess] raytracer done, len=%zu direct_amp=%.6f\n",
            raytracer_processed.size(), direct_amp); fflush(stderr);

    const auto make_iterator = [](auto it) {
        return util::make_mapping_iterator_adapter(std::move(it),
                                                   max_frequency_functor{});
    };

    //  For HRTF capsules, skip per-channel normalization and decay enforcement.
    //  These must be consistent between left/right ears to preserve the
    //  interaural level difference (ILD) — a critical binaural cue.
    //  The global normalization in threaded_engine handles level correctly.
    constexpr bool is_hrtf =
            std::is_same<std::decay_t<Method>,
                         core::attenuator::hrtf>::value;

    const auto src_recv_distance = static_cast<float>(
            glm::distance(source_position, receiver_position));

    if (input.waveguide.empty()) {
        //  Raytracer-only: DC-block, analytical tail extension, distance-normalize.
        dc_block(raytracer_processed, output_sample_rate);
        extend_with_analytical_tail(raytracer_processed, output_sample_rate,
                                     room_volume, environment.speed_of_sound);
        normalize_distance(raytracer_processed, output_sample_rate,
                           src_recv_distance, direct_amp);
        trim_leading_silence(raytracer_processed, output_sample_rate);
        return raytracer_processed;
    }

    //  DC-block both signals.
    dc_block(waveguide_processed, output_sample_rate);
    dc_block(raytracer_processed, output_sample_rate);

    //  Temporal alignment: shift signals so their direct sound onsets match.
    //  The waveguide and raytracer have different propagation models that can
    //  produce slightly different onset times.  Misalignment causes destructive
    //  interference in the crossover band, producing a notch at the blend
    //  frequency and an audible "hollow" coloration.
    align_peaks(waveguide_processed, raytracer_processed);

    //  Safe: input.waveguide guaranteed non-empty (empty case returned above).
    assert(!input.waveguide.empty());
    const auto cutoff = *std::max_element(make_iterator(begin(input.waveguide)),
                                          make_iterator(end(input.waveguide))) /
                        output_sample_rate;
    constexpr auto width = 0.3;

    fprintf(stderr,
            "[combined::postprocess] time-freq substitution: cutoff=%.4f "
            "(%.0f Hz) width=%.2f\n",
            cutoff, cutoff * output_sample_rate, width);
    fflush(stderr);

    //  ── Time-frequency substitution ────────────────────────────────────
    //  The raytracer spectral synthesis now has correct energy at ALL
    //  frequencies.  Use it as the primary signal.  Blend in the
    //  waveguide's low-frequency contribution only for the early part
    //  (where its phase accuracy for room modes / diffraction matters).

    //  Low-pass filter both signals to isolate low-frequency components.
    const auto lp_fft_len = frequency_domain::best_fft_length(
            std::max(waveguide_processed.size(),
                     raytracer_processed.size())) << 2;

    auto wg_lp = waveguide_processed;
    {
        frequency_domain::filter f{lp_fft_len};
        f.run(begin(wg_lp), end(wg_lp), begin(wg_lp),
              [&](auto cplx, auto freq) {
                  return cplx * static_cast<float>(
                          frequency_domain::compute_lopass_magnitude(
                                  freq, cutoff, width, 0));
              });
    }

    auto rt_lp = raytracer_processed;
    {
        frequency_domain::filter f{lp_fft_len};
        f.run(begin(rt_lp), end(rt_lp), begin(rt_lp),
              [&](auto cplx, auto freq) {
                  return cplx * static_cast<float>(
                          frequency_domain::compute_lopass_magnitude(
                                  freq, cutoff, width, 0));
              });
    }

    //  Spectral level-match: correct the waveguide low-freq spectrum to
    //  match the raytracer's spectral shape in the crossover band.
    //  A single broadband gain can't correct spectral tilt (e.g. waveguide
    //  has too much bass, too little mid relative to raytracer).  Instead,
    //  compute a smooth spectral correction curve and apply it.
    //
    //  Measurement window adapts to room size (mixing_time-based).
    {
        const auto measure_len = std::min(wg_lp.size(), rt_lp.size());
        const auto early_end = std::min(
                measure_len,
                static_cast<size_t>(
                        (mixing_time * 2.0 + 0.05) * output_sample_rate));

        //  Broadband level-match first (coarse correction).
        double wg_ss = 0.0, rt_ss = 0.0;
        for (size_t i = 0; i < early_end; ++i) {
            wg_ss += static_cast<double>(wg_lp[i]) * wg_lp[i];
            rt_ss += static_cast<double>(rt_lp[i]) * rt_lp[i];
        }
        const auto wg_rms = std::sqrt(wg_ss / std::max(early_end, size_t{1}));
        const auto rt_rms = std::sqrt(rt_ss / std::max(early_end, size_t{1}));
        const auto gain = (wg_rms > 1e-10)
                ? static_cast<float>(rt_rms / wg_rms)
                : 1.0f;
        for (auto& s : wg_lp) s *= gain;

        //  Fine spectral correction: compute spectral ratio in the
        //  crossover band and apply a smoothed correction curve.
        //  This fixes spectral tilt differences between the two methods.
        const auto corr_fft_len = frequency_domain::best_fft_length(
                wg_lp.size()) << 2;
        const auto corr_bins = corr_fft_len / 2 + 1;
        std::vector<float> wg_mag(corr_bins, 0.0f);
        std::vector<float> rt_mag(corr_bins, 0.0f);

        //  Capture spectra of the early portion of both signals.
        {
            frequency_domain::filter spec_filt{corr_fft_len};
            size_t k = 0;
            spec_filt.run(begin(wg_lp), begin(wg_lp) + early_end,
                          begin(wg_lp),  //  pass-through (capture mags only)
                          [&](auto cplx, auto /*freq*/) {
                              wg_mag[k++] = std::abs(cplx);
                              return cplx;
                          });
            k = 0;
            spec_filt.run(begin(rt_lp), begin(rt_lp) + early_end,
                          begin(rt_lp),
                          [&](auto cplx, auto /*freq*/) {
                              rt_mag[k++] = std::abs(cplx);
                              return cplx;
                          });
        }

        //  Compute smoothed spectral correction (ratio of RT/WG magnitudes).
        //  Smooth with a 16-bin running average (half the old 32) for finer
        //  spectral resolution in the correction curve.
        std::vector<float> correction(corr_bins, 1.0f);
        constexpr size_t smooth_hw = 8;
        for (size_t k = 0; k < corr_bins; ++k) {
            const auto lo = (k > smooth_hw) ? k - smooth_hw : size_t{0};
            const auto hi = std::min(corr_bins, k + smooth_hw + 1);
            double sum_rt = 0.0, sum_wg = 0.0;
            for (size_t j = lo; j < hi; ++j) {
                sum_rt += rt_mag[j];
                sum_wg += wg_mag[j];
            }
            if (sum_wg > 1e-10) {
                //  Clamp correction to ±6 dB to prevent overcorrection.
                //  The old ±12 dB range was too aggressive and could distort
                //  the room's natural spectral character.
                const auto ratio = static_cast<float>(sum_rt / sum_wg);
                correction[k] = std::clamp(ratio, 0.25f, 4.0f);
            }
        }

        //  Apply spectral correction to the full waveguide low-pass signal.
        {
            frequency_domain::filter corr_filt{corr_fft_len};
            size_t k = 0;
            corr_filt.run(begin(wg_lp), end(wg_lp), begin(wg_lp),
                          [&](auto cplx, auto /*freq*/) {
                              const auto ci = std::min(k, corr_bins - 1);
                              ++k;
                              return cplx * correction[ci];
                          });
        }

        fprintf(stderr,
                "[combined::postprocess] spectral level-match: broadband "
                "gain=%.3f (wg_rms=%.6f rt_rms=%.6f)\n",
                gain, wg_rms, rt_rms);
        fflush(stderr);
    }

    //  Room-dependent mixing time for waveguide blend.
    //  Use mixing_time (Kuttruff) for the early/late transition, with a
    //  fade duration of 2× mixing time for a smoother transition that
    //  reduces energy discontinuities at the blend boundary.
    const auto mix_samples = static_cast<size_t>(mixing_time * output_sample_rate);
    const auto fade_samples = static_cast<size_t>(2.0 * mixing_time * output_sample_rate);

    auto filtered = raytracer_processed;  //  Full-spectrum base signal.
    const auto blend_len = std::min({filtered.size(), wg_lp.size(), rt_lp.size()});

    for (size_t i = 0; i < blend_len; ++i) {
        float w = 1.0f;
        if (i >= mix_samples + fade_samples) {
            w = 0.0f;
        } else if (i >= mix_samples) {
            const auto t = static_cast<float>(i - mix_samples) /
                           static_cast<float>(fade_samples);
            w = 0.5f * (1.0f + std::cos(static_cast<float>(M_PI) * t));
        }
        if (w > 0.0f) {
            filtered[i] += w * (wg_lp[i] - rt_lp[i]);
        }
    }

    fprintf(stderr,
            "[combined::postprocess] blend: mix=%.0fms fade=%.0fms (room-dependent)\n",
            1000.0 * mix_samples / output_sample_rate,
            1000.0 * fade_samples / output_sample_rate);
    fflush(stderr);

    //  NO onset ramp — the direct sound spike must be preserved.
    //  DC offset is already handled by dc_block above.

    //  Enforce decay only on the diffuse tail (after mixing time).
    //  Skip for HRTF: independent enforcement per ear can desync the
    //  inter-channel envelope and corrupt binaural coherence.
    if constexpr (!is_hrtf) {
        enforce_decay(filtered, output_sample_rate, mixing_time);
    }

    //  Gentle tail fade (2%) to suppress any remaining edge artifacts.
    {
        const auto fade_len = std::max(
                size_t{256},
                static_cast<size_t>(filtered.size() * 0.02));
        const auto fade_start = filtered.size() > fade_len
                ? filtered.size() - fade_len : 0;
        for (size_t i = fade_start; i < filtered.size(); ++i) {
            const auto t = static_cast<float>(i - fade_start) /
                           static_cast<float>(fade_len);
            filtered[i] *= 0.5f * (1.0f + std::cos(static_cast<float>(M_PI) * t));
        }
    }

    //  Analytical tail extension (replaces FDN).
    //  Extends the IR with exponentially-decaying noise calibrated to the
    //  signal's own decay rate.  Physically motivated by Sabine/Eyring theory.
    extend_with_analytical_tail(filtered, output_sample_rate,
                                 room_volume, environment.speed_of_sound);

    //  Distance-based normalization: direct sound peak = 1/distance.
    //  Preserves physical room gain and distance cue.
    //  For HRTF: both ears get the same distance-based scale, preserving ILD
    //  since the HRTF directional weighting is already baked into the signal.
    normalize_distance(filtered, output_sample_rate, src_recv_distance,
                       direct_amp);
    trim_leading_silence(filtered, output_sample_rate);

    return filtered;
}

}  // namespace combined
}  // namespace wayverb
