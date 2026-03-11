#pragma once

#include "combined/fdn.h"

#include "raytracer/postprocess.h"

#include "waveguide/canonical.h"
#include "waveguide/config.h"
#include "waveguide/dispersion_compensation.h"
#include "waveguide/postprocess.h"

#include "core/sinc.h"
#include "core/sum_ranges.h"

#include "audio_file/audio_file.h"

#include <algorithm>
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
//  IR quality helpers
////////////////////////////////////////////////////////////////////////////////

//  Compute mixing time from room volume (Kuttruff formula).
//  Must match the value used in stochastic synthesis.
inline double compute_mixing_time(double room_volume, double speed_of_sound) {
    const auto t_mix = std::sqrt(
            room_volume / (4.0 * M_PI * std::pow(speed_of_sound, 3.0)));
    return std::max(0.01, std::min(t_mix, 0.5));
}

//  Enforce monotonic energy decay after the mixing time.
//  Only acts on the late/diffuse portion of the IR — early reflections
//  (before mixing time) are left untouched since they naturally have
//  variable amplitude.
template <typename Vec>
void enforce_decay(Vec& v, double sample_rate, double mixing_time) {
    if (v.size() < 2048) return;

    //  Use 2048-sample windows (46ms at 44.1kHz) so individual reflections
    //  don't trigger false decay enforcement.
    constexpr size_t win = 2048;
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
template <typename Vec>
void normalize_distance(Vec& v, double sample_rate, float distance) {
    if (v.empty() || distance < 0.01f) return;

    const float target = 1.0f / std::max(distance, 0.1f);

    //  Find the direct sound peak in the first 100ms.
    const size_t search_len = std::min(v.size(),
            static_cast<size_t>(0.1 * sample_rate));
    float direct_peak = 0.0f;
    for (size_t i = 0; i < search_len; ++i) {
        direct_peak = std::max(direct_peak, std::abs(v[i]));
    }

    //  Fall back to global peak if direct sound is too weak.
    float global_peak = 0.0f;
    for (const auto& s : v) {
        global_peak = std::max(global_peak, std::abs(s));
    }

    const float ref = (direct_peak > global_peak * 0.05f)
            ? direct_peak : global_peak;

    if (ref > 1e-10f) {
        const auto scale = target / ref;
        for (auto& s : v) {
            s *= scale;
        }
        fprintf(stderr,
                "[normalize_distance] d=%.2fm target=%.3f direct_peak=%.6f "
                "scale=%.3f\n",
                distance, target, direct_peak, scale);
        fflush(stderr);
    }
}

//  DC-block a signal with a high-pass filter.
template <typename Vec>
void dc_block(Vec& v, double sample_rate, double cutoff_hz = 20.0,
              double width = 0.3) {
    if (v.empty()) return;
    const auto cutoff = cutoff_hz / sample_rate;
    frequency_domain::filter filt{
            frequency_domain::best_fft_length(v.size()) << 2};
    filt.run(begin(v), end(v), begin(v), [&](auto cplx, auto freq) {
        return cplx * static_cast<float>(
                frequency_domain::compute_hipass_magnitude(
                        freq, cutoff, width, 0));
    });
}

//  Temporal alignment: find first significant peak in each signal and shift
//  so their direct sounds are aligned.
template <typename VecA, typename VecB>
void align_peaks(VecA& a, VecB& b) {
    if (a.empty() || b.empty()) return;

    const auto find_onset = [](const auto& v) -> size_t {
        float peak = 0.0f;
        for (const auto& s : v) peak = std::max(peak, std::abs(s));
        const float thresh = peak * 0.1f;
        for (size_t i = 0; i < v.size(); ++i) {
            if (std::abs(v[i]) >= thresh) return i;
        }
        return 0;
    };

    const auto onset_a = find_onset(a);
    const auto onset_b = find_onset(b);

    if (onset_a == onset_b) return;

    const auto shift = [](auto& v, size_t amount) {
        if (amount == 0 || amount >= v.size()) return;
        for (size_t i = v.size() - 1; i >= amount; --i) {
            v[i] = v[i - amount];
        }
        for (size_t i = 0; i < amount; ++i) {
            v[i] = 0.0f;
        }
    };

    if (onset_a < onset_b) {
        const auto delta = onset_b - onset_a;
        shift(a, delta);
        fprintf(stderr, "[align_peaks] shifted waveguide forward by %zu samples\n", delta);
    } else {
        const auto delta = onset_a - onset_b;
        shift(b, delta);
        fprintf(stderr, "[align_peaks] shifted raytracer forward by %zu samples\n", delta);
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

                 const auto safe_cutoff =
                         std::max(static_cast<float>(cutoff), 1e-6f);
                 const auto safe_width =
                         std::max(static_cast<float>(width), 0.05f);
                 const auto t = 0.5f + 0.5f * std::tanh(
                         3.0f * (static_cast<float>(freq) - safe_cutoff) /
                         (safe_width * safe_cutoff));

                 const auto unit_lo = std::polar(1.0f - t, phase_lo);
                 const auto unit_hi = std::polar(t, phase_hi);
                 const auto phasor_sum = unit_lo + unit_hi;

                 float blended_phase;
                 if (std::abs(phasor_sum) > 0.05f) {
                     blended_phase = std::arg(phasor_sum);
                 } else {
                     blended_phase = phase_lo;
                 }

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
    //  for the 7-point stencil's frequency-dependent phase velocity error.
    //  This shifts modal peaks back to their true frequencies and reduces
    //  high-frequency smearing in the spectrogram.
    if (!input.waveguide.empty()) {
        const auto wg_sr = input.waveguide.front().band.sample_rate;
        waveguide::compensate_dispersion(
                waveguide_processed, wg_sr, output_sample_rate);
    }

    fprintf(stderr, "[combined::postprocess] raytracer postprocess...\n"); fflush(stderr);
    auto raytracer_processed = raytracer::postprocess(input.raytracer,
                                                       method,
                                                       receiver_position,
                                                       room_volume,
                                                       environment,
                                                       output_sample_rate);
    fprintf(stderr, "[combined::postprocess] raytracer done, len=%zu\n",
            raytracer_processed.size()); fflush(stderr);

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
        //  Raytracer-only: DC-block, FDN tail extension, distance-normalize.
        dc_block(raytracer_processed, output_sample_rate);
        {
            const double sig_dur = static_cast<double>(
                    raytracer_processed.size()) / output_sample_rate;
            const double ext = fdn::compute_fdn_extension(
                    sig_dur, room_volume, environment.speed_of_sound);
            fdn::extend_with_fdn(raytracer_processed, output_sample_rate,
                                 room_volume, environment.speed_of_sound, ext);
        }
        normalize_distance(raytracer_processed, output_sample_rate,
                           src_recv_distance);
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
        //  Smooth with a 32-bin running average to avoid noise in the ratio.
        std::vector<float> correction(corr_bins, 1.0f);
        constexpr size_t smooth_hw = 16;
        for (size_t k = 0; k < corr_bins; ++k) {
            const auto lo = (k > smooth_hw) ? k - smooth_hw : size_t{0};
            const auto hi = std::min(corr_bins, k + smooth_hw + 1);
            double sum_rt = 0.0, sum_wg = 0.0;
            for (size_t j = lo; j < hi; ++j) {
                sum_rt += rt_mag[j];
                sum_wg += wg_mag[j];
            }
            if (sum_wg > 1e-10) {
                //  Clamp correction to ±12 dB to prevent overcorrection.
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
    //  fade duration equal to the mixing time itself.
    const auto mix_samples = static_cast<size_t>(mixing_time * output_sample_rate);
    const auto fade_samples = static_cast<size_t>(mixing_time * output_sample_rate);

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
            filtered[i] *= 0.5f * (1.0f + std::cos(3.14159265f * t));
        }
    }

    //  FDN late-reverb tail extension.
    //  Extends the IR with a smooth exponentially-decaying diffuse tail
    //  generated by a 16-line Feedback Delay Network.
    {
        const double sig_dur = static_cast<double>(
                filtered.size()) / output_sample_rate;
        const double ext = fdn::compute_fdn_extension(
                sig_dur, room_volume, environment.speed_of_sound);
        fdn::extend_with_fdn(filtered, output_sample_rate,
                             room_volume, environment.speed_of_sound, ext);
    }

    //  Distance-based normalization: direct sound peak = 1/distance.
    //  Preserves physical room gain and distance cue.
    //  For HRTF: both ears get the same distance-based scale, preserving ILD
    //  since the HRTF directional weighting is already baked into the signal.
    normalize_distance(filtered, output_sample_rate, src_recv_distance);

    return filtered;
}

}  // namespace combined
}  // namespace wayverb
