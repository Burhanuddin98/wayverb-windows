#pragma once

#include "raytracer/postprocess.h"

#include "waveguide/canonical.h"
#include "waveguide/config.h"
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

//  Normalize by direct sound amplitude (first significant peak).
//  This preserves the physical ratio between direct and reverberant energy,
//  which determines C80, D50, and other acoustic metrics.
//  Falls back to peak normalization if no clear direct sound is found.
template <typename Vec>
void normalize_direct_sound(Vec& v, float target = 0.9f) {
    if (v.empty()) return;

    //  Find the first significant peak (direct sound) in the first 10ms.
    //  Direct sound should arrive within the first few milliseconds.
    const size_t search_len = std::min(v.size(), size_t{4410});  //  ~100ms @ 44.1k
    float direct_peak = 0.0f;
    for (size_t i = 0; i < search_len; ++i) {
        direct_peak = std::max(direct_peak, std::abs(v[i]));
    }

    //  If direct sound is too weak, fall back to global peak.
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

    if (input.waveguide.empty()) {
        //  Raytracer-only: DC-block, normalize (mono only), return.
        dc_block(raytracer_processed, output_sample_rate);
        if constexpr (!is_hrtf) {
            normalize_direct_sound(raytracer_processed);
        }
        return raytracer_processed;
    }

    //  DC-block both signals.
    dc_block(waveguide_processed, output_sample_rate);
    dc_block(raytracer_processed, output_sample_rate);

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

    //  Level-match waveguide low-freq to raytracer low-freq.
    {
        const auto measure_len = std::min(wg_lp.size(), rt_lp.size());
        const auto early_end = std::min(
                measure_len,
                static_cast<size_t>(0.1 * output_sample_rate));
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
        fprintf(stderr,
                "[combined::postprocess] level-match gain=%.3f "
                "(wg_rms=%.6f rt_rms=%.6f)\n",
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

    //  Normalize by direct sound amplitude — preserves physical ratios
    //  (C80, D50, etc.) instead of destroying them with peak normalization.
    //  Skip for HRTF: per-channel normalization destroys the interaural
    //  level difference (ILD), making the binaural output sound mono.
    //  The global normalization in threaded_engine preserves ILD correctly.
    if constexpr (!is_hrtf) {
        normalize_direct_sound(filtered);
    }

    return filtered;
}

}  // namespace combined
}  // namespace wayverb
