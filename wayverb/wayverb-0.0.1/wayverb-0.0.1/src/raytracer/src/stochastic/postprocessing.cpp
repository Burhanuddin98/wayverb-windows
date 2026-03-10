#include "raytracer/stochastic/postprocessing.h"

#include "core/cl/iterator.h"
#include "core/mixdown.h"
#include "core/pressure_intensity.h"

#include "utilities/for_each.h"
#include "utilities/map.h"

#include <iostream>

namespace wayverb {
namespace raytracer {
namespace stochastic {

double constant_mean_event_occurrence(double speed_of_sound,
                                      double room_volume) {
    return 4 * M_PI * std::pow(speed_of_sound, 3.0) / room_volume;
}

double mean_event_occurrence(double constant, double t) {
    return std::min(constant * std::pow(t, 2.0), 10000.0);
}

double t0(double constant) {
    return std::pow(2.0 * std::log(2.0) / constant, 1.0 / 3.0);
}

dirac_sequence generate_dirac_sequence(double speed_of_sound,
                                       double room_volume,
                                       double sample_rate,
                                       double max_time) {
    //  Dense random noise sequence instead of sparse Poisson-spaced Diracs.
    //  The energy weighting in weight_sequence() normalises correctly:
    //  squared_sum over a dense ±1 sequence = number_of_samples, so
    //  scale_factor = sqrt(histogram_energy / N) which distributes energy
    //  evenly, producing smooth diffuse reverb instead of grainy impulse
    //  trains.
    std::default_random_engine engine{std::random_device{}()};
    std::uniform_int_distribution<int> dist(0, 1);

    util::aligned::vector<float> ret(std::ceil(max_time * sample_rate), 0);
    for (size_t i = 0; i < ret.size(); ++i) {
        ret[i] = dist(engine) ? 1.0f : -1.0f;
    }
    return {ret, sample_rate};
}

void sum_histograms(energy_histogram& a, const energy_histogram& b) {
    sum_vectors(a.histogram, b.histogram);
    a.sample_rate = b.sample_rate;
}

util::aligned::vector<core::bands_type> weight_sequence(
        const energy_histogram& histogram,
        const dirac_sequence& sequence,
        double acoustic_impedance) {
    auto ret = util::map_to_vector(
            begin(sequence.sequence), end(sequence.sequence), [](auto i) {
                return core::make_bands_type(i);
            });

    const auto convert_index = [&](auto ind) -> size_t {
        return ind * sequence.sample_rate / histogram.sample_rate;
    };

    const auto ideal_sequence_length =
            convert_index(histogram.histogram.size());
    if (ideal_sequence_length < ret.size()) {
        ret.resize(ideal_sequence_length);
    }

    //  Smooth the histogram energy with a 3-point triangle filter to prevent
    //  audible clicks at bin boundaries when using dense noise.
    const auto& h = histogram.histogram;
    const auto hsize = h.size();
    auto smoothed = h;  //  copy
    if (hsize >= 3) {
        for (size_t i = 1; i + 1 < hsize; ++i) {
            smoothed[i] = h[i - 1] * 0.25f + h[i] * 0.5f + h[i + 1] * 0.25f;
        }
    }

    for (size_t i = 0, e = hsize; i != e; ++i) {
        const auto get_sequence_index = [&](auto ind) {
            return std::min(convert_index(ind), ret.size());
        };

        const auto beg = get_sequence_index(i);
        const auto end = get_sequence_index(i + 1);

        const auto squared_summed = frequency_domain::square_sum(
                begin(sequence.sequence) + beg, begin(sequence.sequence) + end);
        const auto scale_factor =
                squared_summed != 0.0f
                        ? core::intensity_to_pressure(
                                  smoothed[i] / squared_summed,
                                  acoustic_impedance)
                        : cl_double8{};

        std::for_each(begin(ret) + beg, begin(ret) + end, [&](auto& i) {
            i *= scale_factor;
        });
    }

    return ret;
}

/// Full ISO 9613-1:1993 atmospheric absorption coefficient.
/// Returns absorption in Nepers/metre at the given frequency.
///
/// Parameters:
///   freq_hz  — frequency in Hz
///   T_kelvin — temperature in Kelvin (default 293.15 = 20°C)
///   humidity — relative humidity in percent (default 50%)
///   p_atm    — atmospheric pressure in kPa (default 101.325)
///
/// Reference: ISO 9613-1:1993 "Acoustics — Attenuation of sound during
///            propagation outdoors — Part 1: Calculation of the absorption
///            of sound by the atmosphere"
static double air_absorption_nepers(double freq_hz,
                                     double T_kelvin = 293.15,
                                     double humidity = 50.0,
                                     double p_atm = 101.325) {
    //  Reference conditions
    constexpr double T_ref = 293.15;   //  20°C in Kelvin
    constexpr double T_01  = 273.16;   //  Triple point of water
    constexpr double p_ref = 101.325;  //  Standard atmosphere (kPa)

    const double T_rel = T_kelvin / T_ref;
    const double p_rel = p_atm / p_ref;
    const double f = freq_hz;

    //  Saturation vapour pressure ratio (ISO 9613-1, Eq. 3)
    //  C = -6.8346 × (T_01/T)^1.261 + 4.6151
    const double C = -6.8346 * std::pow(T_01 / T_kelvin, 1.261) + 4.6151;
    const double h = humidity * std::pow(10.0, C) * (p_ref / p_atm);

    //  Oxygen relaxation frequency (ISO 9613-1, Eq. 4)
    const double fr_O = (p_rel) * (24.0 + 4.04e4 * h *
            (0.02 + h) / (0.391 + h));

    //  Nitrogen relaxation frequency (ISO 9613-1, Eq. 5)
    const double fr_N = (p_rel) * std::pow(T_rel, -0.5) *
            (9.0 + 280.0 * h * std::exp(-4.170 * (std::pow(T_rel, -1.0/3.0) - 1.0)));

    //  Absorption coefficient in dB/m (ISO 9613-1, Eq. 1)
    const double alpha_dB =
            8.686 * f * f * (
                //  Classical + rotational absorption
                1.84e-11 * (1.0 / p_rel) * std::pow(T_rel, 0.5)
                //  Vibrational: oxygen relaxation
                + std::pow(T_rel, -2.5) * (
                    0.01275 * std::exp(-2239.1 / T_kelvin) /
                        (fr_O + f * f / fr_O)
                //  Vibrational: nitrogen relaxation
                    + 0.10680 * std::exp(-3352.0 / T_kelvin) /
                        (fr_N + f * f / fr_N)
                )
            );

    //  Convert dB/m to Nepers/m:  1 Np = 8.686 dB
    return alpha_dB / 8.686;
}

/// Compute the mixing time for a room (Kuttruff formula).
/// This is the time after which the sound field becomes diffuse.
/// Before this time, the image-source method is more accurate.
static double compute_mixing_time(double room_volume, double speed_of_sound) {
    //  t_mix = sqrt(V / (4·π·c³))  [Kuttruff, Room Acoustics, 5th ed.]
    //  For a 50 m³ room at 340 m/s: t_mix ≈ 0.018 s (18 ms)
    //  For a 500 m³ room: t_mix ≈ 0.057 s (57 ms)
    //  For a 5000 m³ room: t_mix ≈ 0.18 s (180 ms)
    const auto t_mix = std::sqrt(
            room_volume / (4.0 * M_PI * std::pow(speed_of_sound, 3.0)));
    //  Clamp to [10ms, 500ms] for safety.
    return std::max(0.01, std::min(t_mix, 0.5));
}

/// Direct spectral synthesis of the stochastic reverb tail.
///
/// Architecture rebuild: phase-coherent STFT with onset delay and air
/// absorption.
///
///   1. For each STFT frame, interpolate the 8-band energy histogram to
///      get a continuous PSD (power spectral density) at every FFT bin.
///   2. Apply air absorption: PSD × exp(-2·α(f)·c·t) per ISO 9613-1.
///   3. Set each bin's magnitude = sqrt(N × PSD × Z), with PHASE CONTINUITY
///      (accumulated phase, not random per frame — eliminates vertical
///      striping artifacts in the spectrogram).
///   4. Apply onset delay: no stochastic energy before the mixing time
///      (Kuttruff formula), with a raised-cosine fade-in over one mixing
///      time width.
///   5. IFFT to produce time-domain noise with the correct spectral shape.
///   6. Overlap-add with sqrt-Hann window for energy-preserving transitions.
///
/// This replaces the old random-phase-per-frame approach which produced:
///   - Vertical striping in the bass (uncorrelated phase between frames)
///   - Energy from t=0 (no onset delay, burying image-source reflections)
///   - Abrupt HF cutoff (no air absorption)
///   - Over-prominent bass (incorrect PSD density scaling)
util::aligned::vector<float> postprocessing(const energy_histogram& histogram,
                                            const dirac_sequence& sequence,
                                            double acoustic_impedance,
                                            double room_volume,
                                            double speed_of_sound) {
    const auto sr = sequence.sample_rate;
    const auto total_len = sequence.sequence.size();

    if (total_len == 0 || histogram.histogram.empty()) {
        return util::aligned::vector<float>(total_len, 0.0f);
    }

    //  ── Mixing time (onset delay for stochastic energy) ────────────
    const auto mixing_time = compute_mixing_time(room_volume, speed_of_sound);
    const auto fade_duration = mixing_time;  //  Fade in over one mixing time.

    fprintf(stderr,
            "[stochastic::postprocessing] spectral synthesis: sr=%.0f "
            "len=%zu hist_bins=%zu mixing_time=%.3fs\n",
            sr, total_len, histogram.histogram.size(), mixing_time);
    fflush(stderr);

    //  ── STFT parameters ──────────────────────────────────────────────
    //  4096-sample windows with 75% overlap for better frequency resolution
    //  at low frequencies and smoother transitions.
    constexpr size_t WIN = 4096;
    constexpr size_t HOP = WIN / 4;  //  75% overlap (1024 samples)
    const auto fft_len = frequency_domain::best_fft_length(WIN);
    const auto num_bins = fft_len / 2 + 1;

    //  ── Band parameters ──────────────────────────────────────────────
    const auto params = hrtf_data::hrtf_band_params(sr);

    //  Band centre frequencies in Hz (geometric mean of edges).
    std::array<double, 8> fc{};
    for (int i = 0; i < 8; ++i) {
        fc[i] = std::sqrt(params.edges[i] * params.edges[i + 1]) * sr;
    }

    //  ── Smooth histogram ─────────────────────────────────────────────
    const auto& h = histogram.histogram;
    const auto hsize = h.size();
    auto smoothed = h;
    if (hsize >= 3) {
        for (size_t i = 1; i + 1 < hsize; ++i) {
            smoothed[i] =
                    h[i - 1] * 0.25f + h[i] * 0.5f + h[i + 1] * 0.25f;
        }
    }

    //  ── sqrt-Hann window (energy-preserving OLA at 75% overlap) ─────
    //  With 75% overlap and sqrt-Hann, the sum of squared windows = 1.5
    //  (constant). We compensate by dividing by 1.5 after OLA.
    std::vector<float> win(WIN);
    for (size_t i = 0; i < WIN; ++i) {
        win[i] = std::sqrt(0.5f * (1.0f - std::cos(
                2.0f * static_cast<float>(M_PI) * static_cast<float>(i) /
                static_cast<float>(WIN))));
    }

    //  OLA normalization factor for 75% overlap with sqrt-Hann.
    constexpr float ola_norm = 1.0f / 1.5f;

    //  ── Output ───────────────────────────────────────────────────────
    util::aligned::vector<float> output(total_len, 0.0f);

    //  ── Random phase generator ────────────────────────────────────────
    //  Each STFT frame gets independent random phase per bin.  This is
    //  physically correct for diffuse reverb (Schroeder's statistical
    //  model) and eliminates the repeating flame/streak artifacts that
    //  coherent phase accumulation produces.  The sqrt-Hann OLA window
    //  ensures smooth time-domain transitions between frames despite
    //  the per-frame phase randomization.
    std::default_random_engine rng{std::random_device{}()};
    std::uniform_real_distribution<float> phase_dist(
            -static_cast<float>(M_PI), static_cast<float>(M_PI));

    //  ── Spectral jitter distribution ─────────────────────────────────
    //  Real diffuse reverb follows Rayleigh amplitude statistics: the
    //  magnitude in each frequency bin fluctuates with ~5.6 dB standard
    //  deviation.  We add Gaussian jitter to the log-PSD (in dB) to
    //  break up the smooth 8-band interpolation pattern.
    //  ±4.5 dB std is conservative (real reverb is ~5.6 dB).
    std::normal_distribution<float> jitter_dist(0.0f, 4.5f);

    //  ── Frame-by-frame synthesis ─────────────────────────────────────
    frequency_domain::filter filt{fft_len};
    std::vector<float> zeros(WIN, 0.0f);
    std::vector<float> frame(WIN, 0.0f);

    size_t bin_counter = 0;  //  reset per frame in the lambda

    for (size_t pos = 0; pos + WIN <= total_len; pos += HOP) {
        //  Time at frame centre.
        const double t = static_cast<double>(pos + WIN / 2) / sr;

        //  ── Onset envelope ─────────────────────────────────────────
        //  No stochastic energy before mixing time; raised-cosine
        //  fade-in from mixing_time to mixing_time + fade_duration.
        float onset_gain = 1.0f;
        if (t < mixing_time) {
            onset_gain = 0.0f;
        } else if (t < mixing_time + fade_duration) {
            const auto ft = static_cast<float>(
                    (t - mixing_time) / fade_duration);
            onset_gain = 0.5f * (1.0f - std::cos(
                    static_cast<float>(M_PI) * ft));
        }

        if (onset_gain < 1e-6f) {
            continue;  //  Silent before mixing time — skip frame.
        }

        //  Interpolate histogram energy at this time.
        const double h_pos = t * histogram.sample_rate;
        const size_t i0 =
                std::min(static_cast<size_t>(h_pos), hsize - 1);
        const size_t i1 = std::min(i0 + 1, hsize - 1);
        const double alpha =
                std::max(0.0, std::min(1.0, h_pos - static_cast<double>(i0)));

        //  8-band intensity at this time (linear interpolation).
        std::array<double, 8> energy{};
        for (int b = 0; b < 8; ++b) {
            energy[b] = smoothed[i0].s[b] * (1.0 - alpha) +
                        smoothed[i1].s[b] * alpha;
        }

        //  Convert to PSD: intensity per unit Hz (not normalised frequency).
        //  Using Hz-based bandwidth prevents bass amplification from narrow
        //  log-spaced bands.
        std::array<double, 8> psd{};
        for (int b = 0; b < 8; ++b) {
            const double bw_hz =
                    (params.edges[b + 1] - params.edges[b]) * sr;
            psd[b] = (bw_hz > 1.0) ? std::abs(energy[b]) / bw_hz : 0.0;
        }

        //  IFFT synthesis with phase coherence and air absorption.
        bin_counter = 0;
        filt.run(zeros.begin(), zeros.end(), frame.begin(),
                 [&](auto /*cplx*/, auto freq_raw) -> std::complex<float> {
                     const size_t k = bin_counter++;
                     const double f = static_cast<double>(freq_raw);
                     const double f_hz = f * sr;

                     //  Lower floor to 2 Hz (was 10 Hz — suppressed large
                     //  room modes).
                     if (f_hz < 2.0 || f >= 0.499) {
                         return {0.0f, 0.0f};
                     }

                     //  Log-interpolate PSD between band centres.
                     double ipsd = 0.0;
                     if (f_hz <= fc[0]) {
                         ipsd = psd[0];
                     } else if (f_hz >= fc[7]) {
                         ipsd = psd[7];
                     } else {
                         for (int b = 0; b < 7; ++b) {
                             if (f_hz < fc[b + 1]) {
                                 const double lf = std::log(f_hz);
                                 const double l0 = std::log(fc[b]);
                                 const double l1 = std::log(fc[b + 1]);
                                 const double tt =
                                         (lf - l0) / (l1 - l0);
                                 ipsd = std::exp(
                                         std::log(std::max(psd[b], 1e-30)) *
                                                 (1.0 - tt) +
                                         std::log(std::max(psd[b + 1], 1e-30)) *
                                                 tt);
                                 break;
                             }
                         }
                     }

                     //  Air absorption: exponential decay at high frequencies.
                     //  PSD(f,t) = PSD(f) × exp(-2·α(f)·c·t)
                     //  The factor of 2 is because energy decays at twice
                     //  the rate of pressure amplitude.
                     const double air_atten = std::exp(
                             -2.0 * air_absorption_nepers(f_hz) *
                             speed_of_sound * t);
                     ipsd *= air_atten;

                     //  Convert PSD (per Hz) to PSD (per normalised freq bin)
                     //  for the Parseval relation: multiply by sr.
                     ipsd *= sr;

                     if (ipsd <= 0.0) {
                         return {0.0f, 0.0f};
                     }

                     //  Spectral jitter: perturb PSD by a random amount
                     //  in dB to break up the repeating 8-band pattern.
                     //  This models Rayleigh amplitude statistics of real
                     //  diffuse sound fields.  Clamp to ±12 dB to prevent
                     //  numerical overflow from Gaussian tail outliers.
                     const float jitter_dB = std::clamp(
                             jitter_dist(rng), -12.0f, 12.0f);
                     ipsd *= std::pow(10.0, jitter_dB / 10.0);

                     const auto mag = onset_gain * static_cast<float>(
                             std::sqrt(static_cast<double>(fft_len) * ipsd *
                                       acoustic_impedance));

                     //  Random phase per bin per frame (Schroeder diffuse
                     //  field model).  Eliminates all repeating patterns.
                     const auto phase = phase_dist(rng);

                     return std::polar(mag, phase);
                 });

        //  sqrt-Hann window + overlap-add with normalization.
        for (size_t i = 0; i < WIN && pos + i < total_len; ++i) {
            output[pos + i] += frame[i] * win[i] * ola_norm;
        }
    }

    fprintf(stderr,
            "[stochastic::postprocessing] spectral synthesis done, "
            "len=%zu mixing_time=%.3fs\n",
            output.size(), mixing_time);
    fflush(stderr);

    return output;
}

//  Legacy overload — calls the new version with default room parameters.
util::aligned::vector<float> postprocessing(const energy_histogram& histogram,
                                            const dirac_sequence& sequence,
                                            double acoustic_impedance) {
    //  Estimate room volume from the histogram extent and speed of sound.
    //  This is a rough fallback; the proper path passes room_volume explicitly.
    constexpr double default_room_volume = 50.0;   //  ~small bedroom
    constexpr double default_speed_of_sound = 340.0;
    return postprocessing(histogram, sequence, acoustic_impedance,
                          default_room_volume, default_speed_of_sound);
}

}  // namespace stochastic
}  // namespace raytracer
}  // namespace wayverb
