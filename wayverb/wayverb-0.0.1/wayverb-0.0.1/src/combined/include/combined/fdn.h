#pragma once

/// \file fdn.h
/// Feedback Delay Network (FDN) late reverb tail extension.
///
/// Generates a smooth, exponentially-decaying diffuse reverb tail that
/// extends the stochastic synthesis beyond the ray-traced histogram's
/// duration.  Uses mutually prime delay lengths and a Hadamard feedback
/// matrix for maximal echo density.
///
/// The FDN is excited by sampling the stochastic tail's energy envelope
/// at a handoff point, then producing a seamless continuation.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <random>
#include <vector>

namespace wayverb {
namespace combined {
namespace fdn {

/// Number of delay lines.  16 gives high echo density and matches the
/// 16 simulation bands for per-band decay control.
constexpr size_t kNumLines = 16;

/// Mutually prime delay lengths in samples (at 44100 Hz reference).
/// Chosen to avoid coloring / flutter echoes.  Scaled at runtime for
/// the actual sample rate.
constexpr int kBaseDelays[kNumLines] = {
    1087, 1283, 1447, 1553,
    1699, 1823, 1949, 2089,
    2213, 2357, 2477, 2609,
    2741, 2879, 3001, 3163
};

/// 16x16 Hadamard matrix (normalised).  H16 = H2 ⊗ H2 ⊗ H2 ⊗ H2.
/// Each row/column has ±1/4 entries so ||row||=1 (energy-preserving).
inline std::array<std::array<float, kNumLines>, kNumLines>
make_hadamard_16() {
    std::array<std::array<float, kNumLines>, kNumLines> H{};
    // Build via Kronecker product of H2 = [[1,1],[1,-1]]
    for (size_t i = 0; i < kNumLines; ++i) {
        for (size_t j = 0; j < kNumLines; ++j) {
            int sign = 1;
            size_t a = i, b = j;
            for (int k = 0; k < 4; ++k) {  // log2(16) = 4
                if ((a & 1) && (b & 1)) sign = -sign;
                a >>= 1;
                b >>= 1;
            }
            H[i][j] = static_cast<float>(sign) / 4.0f;  // 1/sqrt(16)
        }
    }
    return H;
}

/// Per-band RT60 estimate from Sabine formula (fallback).
/// Returns approximate RT60 in seconds for each of the 16 half-octave
/// bands.  Uses the room volume and a rough frequency-dependent
/// absorption coefficient (higher frequencies decay faster).
inline std::array<float, kNumLines>
estimate_band_rt60(double room_volume, double speed_of_sound) {
    std::array<float, kNumLines> rt60{};
    constexpr double f0 = 44.0;
    for (size_t b = 0; b < kNumLines; ++b) {
        const double alpha = 0.05 + 0.30 * (static_cast<double>(b) /
                                              (kNumLines - 1));
        const double S = 6.0 * std::pow(room_volume, 2.0 / 3.0);
        rt60[b] = static_cast<float>(
                0.161 * room_volume / std::max(S * alpha, 0.1));
    }
    return rt60;
}

/// Measure per-band RT60 from the actual signal tail.
///
/// Divides the signal into 16 frequency bands via a simple 2nd-order
/// bandpass cascade, measures the energy decay in the last portion of
/// each band, and extrapolates to -60 dB.  Falls back to Sabine
/// estimate for bands where measurement fails (too little energy).
template <typename Vec>
inline std::array<float, kNumLines>
measure_band_rt60(const Vec& signal, double sample_rate,
                  double room_volume, double speed_of_sound) {
    // Start with Sabine as baseline.
    auto rt60 = estimate_band_rt60(room_volume, speed_of_sound);

    if (signal.size() < 8192) return rt60;

    // Band centre frequencies (half-octave spacing from ~44 Hz to ~16 kHz).
    constexpr double f0 = 44.0;
    std::array<double, kNumLines> fc{};
    for (size_t b = 0; b < kNumLines; ++b) {
        fc[b] = f0 * std::pow(2.0, b * 0.5);
    }

    // Use the last 50% of the signal for decay measurement.
    const size_t measure_start = signal.size() / 2;
    const size_t measure_len = signal.size() - measure_start;

    // Window size for energy measurement (4096 samples ≈ 93ms at 44.1kHz).
    constexpr size_t win = 4096;
    const size_t nw = measure_len / win;
    if (nw < 3) return rt60;

    for (size_t b = 0; b < kNumLines; ++b) {
        // 2nd-order IIR bandpass filter (biquad).
        const double w0 = 2.0 * M_PI * fc[b] / sample_rate;
        if (w0 >= M_PI * 0.95) continue;  // Too close to Nyquist.

        const double Q = 1.4;  // Moderate selectivity.
        const double alpha_bp = std::sin(w0) / (2.0 * Q);
        const double b0 =  alpha_bp;
        const double b1 =  0.0;
        const double b2 = -alpha_bp;
        const double a0 =  1.0 + alpha_bp;
        const double a1 = -2.0 * std::cos(w0);
        const double a2 =  1.0 - alpha_bp;

        // Filter the measurement region and compute RMS per window.
        double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
        std::vector<float> wrms(nw, 0.0f);
        for (size_t w = 0; w < nw; ++w) {
            double sum_sq = 0.0;
            for (size_t i = 0; i < win; ++i) {
                const size_t idx = measure_start + w * win + i;
                const double x = static_cast<double>(signal[idx]);
                const double y = (b0 * x + b1 * x1 + b2 * x2
                                  - a1 * y1 - a2 * y2) / a0;
                x2 = x1; x1 = x;
                y2 = y1; y1 = y;
                sum_sq += y * y;
            }
            wrms[w] = static_cast<float>(std::sqrt(sum_sq / win));
        }

        // Linear regression on log-RMS to estimate decay rate.
        // RT60 = -60 dB / (decay_rate_dB_per_sec).
        int valid_count = 0;
        double sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (size_t w = 0; w < nw; ++w) {
            if (wrms[w] < 1e-12f) continue;
            const double t = static_cast<double>(w * win) / sample_rate;
            const double log_rms = 20.0 * std::log10(
                    static_cast<double>(wrms[w]));
            sx += t;
            sy += log_rms;
            sxx += t * t;
            sxy += t * log_rms;
            ++valid_count;
        }

        if (valid_count < 3) continue;

        const double n = static_cast<double>(valid_count);
        const double denom = n * sxx - sx * sx;
        if (std::abs(denom) < 1e-20) continue;

        const double slope = (n * sxy - sx * sy) / denom;  // dB/s

        // Decay slope must be negative (signal decaying).
        if (slope >= -0.1) continue;

        const double measured_rt60 = -60.0 / slope;

        // Sanity check: 0.1s to 30s.
        if (measured_rt60 >= 0.1 && measured_rt60 <= 30.0) {
            rt60[b] = static_cast<float>(measured_rt60);
        }
    }

    fprintf(stderr, "[FDN] measured RT60: ");
    for (size_t b = 0; b < kNumLines; ++b) {
        fprintf(stderr, "%.2f ", rt60[b]);
    }
    fprintf(stderr, "\n");
    fflush(stderr);

    return rt60;
}

/// Feedback Delay Network reverb generator.
///
/// Generates an exponentially-decaying diffuse tail of the specified
/// duration, using the per-band RT60 values for decay shaping.
///
/// \param duration_s  total FDN output duration in seconds
/// \param sample_rate output sample rate
/// \param band_rt60   per-band RT60 in seconds (16 values)
/// \param initial_rms RMS amplitude at the start of the FDN tail
///                    (matched to the end of the stochastic tail)
/// \return mono float vector of FDN reverb tail
inline std::vector<float> generate_fdn_tail(
        double duration_s,
        double sample_rate,
        const std::array<float, kNumLines>& band_rt60,
        float initial_rms) {
    const size_t num_samples = static_cast<size_t>(duration_s * sample_rate);
    if (num_samples == 0 || initial_rms < 1e-12f) return {};

    // Scale delay lengths for the actual sample rate.
    std::array<size_t, kNumLines> delays{};
    for (size_t i = 0; i < kNumLines; ++i) {
        delays[i] = std::max(size_t{1}, static_cast<size_t>(
                kBaseDelays[i] * sample_rate / 44100.0));
    }

    // Per-line circular delay buffers.
    std::vector<std::vector<float>> buffers(kNumLines);
    for (size_t i = 0; i < kNumLines; ++i) {
        buffers[i].resize(delays[i], 0.0f);
    }

    // Write positions (circular).
    std::array<size_t, kNumLines> write_pos{};

    // Per-line feedback gains: g = 10^(-3 * delay / (RT60 * sr))
    // This ensures each delay line decays to -60 dB in RT60 seconds.
    std::array<float, kNumLines> gains{};
    for (size_t i = 0; i < kNumLines; ++i) {
        const double rt = std::max(static_cast<double>(band_rt60[i]), 0.1);
        gains[i] = static_cast<float>(
                std::pow(10.0, -3.0 * static_cast<double>(delays[i]) /
                                (rt * sample_rate)));
    }

    // Hadamard feedback matrix.
    const auto H = make_hadamard_16();

    // Excite the FDN with a short noise burst (first ~10ms).
    std::mt19937 rng{42};
    std::normal_distribution<float> noise{0.0f, 1.0f};
    const size_t excite_len = std::min(
            num_samples,
            static_cast<size_t>(0.01 * sample_rate));

    // Scale excitation to match initial_rms.
    const float excite_scale = initial_rms * std::sqrt(2.0f);

    for (size_t n = 0; n < excite_len; ++n) {
        for (size_t i = 0; i < kNumLines; ++i) {
            buffers[i][write_pos[i]] += excite_scale * noise(rng);
            write_pos[i] = (write_pos[i] + 1) % delays[i];
        }
    }
    // Advance write positions past excitation for lines where
    // excite_len < delay length (they already advanced).

    // Generate output.
    std::vector<float> output(num_samples, 0.0f);
    std::array<float, kNumLines> read_vals{};

    for (size_t n = 0; n < num_samples; ++n) {
        // Read from each delay line.
        for (size_t i = 0; i < kNumLines; ++i) {
            read_vals[i] = buffers[i][write_pos[i]];
        }

        // Mix output: sum all lines (equal contribution).
        float out = 0.0f;
        for (size_t i = 0; i < kNumLines; ++i) {
            out += read_vals[i];
        }
        output[n] = out / static_cast<float>(kNumLines);

        // Feedback: apply Hadamard mixing + per-line gain.
        std::array<float, kNumLines> feedback{};
        for (size_t i = 0; i < kNumLines; ++i) {
            float sum = 0.0f;
            for (size_t j = 0; j < kNumLines; ++j) {
                sum += H[i][j] * read_vals[j];
            }
            feedback[i] = sum * gains[i];
        }

        // Write feedback into delay lines.
        for (size_t i = 0; i < kNumLines; ++i) {
            buffers[i][write_pos[i]] = feedback[i];
            write_pos[i] = (write_pos[i] + 1) % delays[i];
        }
    }

    return output;
}

/// Measure the RMS of a signal in a window around a given sample position.
template <typename Vec>
inline float measure_rms(const Vec& signal,
                         size_t centre,
                         size_t half_window = 2048) {
    if (signal.empty()) return 0.0f;
    const size_t start = (centre > half_window) ? centre - half_window : 0;
    const size_t end = std::min(signal.size(), centre + half_window);
    if (end <= start) return 0.0f;

    double sum = 0.0;
    for (size_t i = start; i < end; ++i) {
        sum += static_cast<double>(signal[i]) * signal[i];
    }
    return static_cast<float>(
            std::sqrt(sum / static_cast<double>(end - start)));
}

/// Extend a reverb tail with FDN-generated late reverb.
///
/// Measures the energy at the end of the existing signal, generates a
/// matching FDN tail, and cross-fades it seamlessly onto the end.
///
/// \param signal       existing IR (modified in-place, may be extended)
/// \param sample_rate  output sample rate
/// \param room_volume  room volume in m³
/// \param speed_of_sound  speed of sound in m/s
/// \param extension_s  how many extra seconds of tail to add
template <typename Vec>
inline void extend_with_fdn(Vec& signal,
                            double sample_rate,
                            double room_volume,
                            double speed_of_sound,
                            double extension_s = 1.0) {
    if (signal.empty() || extension_s <= 0.0) return;

    // Measure the energy at the end of the existing signal.
    const auto tail_rms = measure_rms(signal, signal.size() - 1, 4096);
    if (tail_rms < 1e-10f) {
        fprintf(stderr, "[FDN] tail RMS too low (%.2e), skipping extension\n",
                tail_rms);
        fflush(stderr);
        return;
    }

    // Measure per-band RT60 from the actual signal decay.
    // Falls back to Sabine estimates for bands where measurement fails.
    const auto band_rt60 = measure_band_rt60(
            signal, sample_rate, room_volume, speed_of_sound);

    // Generate FDN tail.
    const auto fdn_tail = generate_fdn_tail(
            extension_s, sample_rate, band_rt60, tail_rms);
    if (fdn_tail.empty()) return;

    // Cross-fade region: last 50ms of existing signal overlaps with
    // first 50ms of FDN tail.
    const size_t xfade_len = std::min(
            static_cast<size_t>(0.05 * sample_rate),
            std::min(signal.size(), fdn_tail.size()));

    const size_t orig_len = signal.size();
    const size_t new_len = orig_len + fdn_tail.size() - xfade_len;
    signal.resize(new_len, 0.0f);

    // Cross-fade in the overlap region.
    for (size_t i = 0; i < xfade_len; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(xfade_len);
        // Raised-cosine crossfade.
        const float w_old = 0.5f * (1.0f + std::cos(3.14159265f * t));
        const float w_new = 1.0f - w_old;
        const size_t sig_idx = orig_len - xfade_len + i;
        signal[sig_idx] = w_old * signal[sig_idx] + w_new * fdn_tail[i];
    }

    // Append the rest of the FDN tail (after crossfade).
    for (size_t i = xfade_len; i < fdn_tail.size(); ++i) {
        signal[orig_len - xfade_len + i] = fdn_tail[i];
    }

    fprintf(stderr,
            "[FDN] extended tail by %.2fs (xfade=%zums, tail_rms=%.4f, "
            "new_len=%zu)\n",
            extension_s,
            static_cast<size_t>(1000.0 * xfade_len / sample_rate),
            tail_rms, new_len);
    fflush(stderr);
}

/// Compute how much FDN extension is needed based on Sabine RT60
/// vs the existing signal duration.
inline double compute_fdn_extension(double signal_duration_s,
                                    double room_volume,
                                    double speed_of_sound) {
    // Sabine RT60 estimate (rough, using average alpha ~0.15).
    const double S = 6.0 * std::pow(room_volume, 2.0 / 3.0);
    const double rt60 = 0.161 * room_volume / std::max(S * 0.15, 0.1);

    // We want the total IR to be at least RT60 long.
    // If the stochastic tail already covers it, add a minimal extension
    // (0.5s) for a smoother fade-out.  Otherwise add the difference.
    const double deficit = rt60 - signal_duration_s;
    if (deficit > 0.0) {
        return std::min(deficit + 0.5, 10.0);  // cap at 10s extension
    }
    return std::min(0.5, 10.0);  // minimal smoothing extension
}

}  // namespace fdn
}  // namespace combined
}  // namespace wayverb
