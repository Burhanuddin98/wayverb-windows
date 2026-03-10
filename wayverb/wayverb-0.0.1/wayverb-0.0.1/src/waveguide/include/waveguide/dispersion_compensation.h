#pragma once

/// \file dispersion_compensation.h
/// Post-processing frequency warping to compensate for the numerical
/// dispersion of the 7-point rectilinear FDTD stencil.
///
/// The 3D 7-point scheme at the Courant limit (lambda = 1/sqrt(3)) has
/// the axis-aligned dispersion relation:
///   cos(omega_num * dt) = 2/3 + cos(k*h) / 3
/// True relation: omega = c * k.
///
/// High-frequency waves propagate slower numerically, causing modal
/// frequencies to shift downward and spectrograms to appear smeared.
/// This filter stretches the spectrum back to the correct frequencies.

#include "frequency_domain/multiband_filter.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace wayverb {
namespace waveguide {

/// Apply dispersion compensation to a waveguide output signal.
///
/// \param signal          waveguide output (at output_sample_rate), modified
/// \param waveguide_sr    native waveguide sampling rate (1/dt)
/// \param output_sr       output sampling rate
template <typename Vec>
inline void compensate_dispersion(Vec& signal,
                                  double waveguide_sr,
                                  double output_sr) {
    if (signal.empty() || waveguide_sr <= 0.0 || output_sr <= 0.0) return;

    //  Only apply if there's meaningful bandwidth to correct.
    //  The warping is negligible below ~10% of waveguide Nyquist.
    const double nyquist_ratio = (output_sr * 0.5) / (waveguide_sr * 0.5);
    if (nyquist_ratio < 0.05) return;

    const auto fft_len =
            frequency_domain::best_fft_length(signal.size()) << 2;
    const auto num_bins = fft_len / 2 + 1;

    //  Pass 1: capture the spectrum.
    std::vector<std::complex<float>> spectrum(num_bins);
    size_t idx = 0;
    frequency_domain::filter filt{fft_len};
    filt.run(begin(signal), end(signal), begin(signal),
             [&](auto cplx, auto /*freq*/) {
                 if (idx < num_bins) spectrum[idx] = cplx;
                 ++idx;
                 return cplx;
             });

    //  Pass 2: resample spectrum with dispersion compensation.
    //
    //  For each output bin at true frequency f_true, compute the
    //  numerical frequency f_num where that content actually ended up
    //  in the dispersed waveguide output, and interpolate from there.
    //
    //  Inverse warp (true -> numerical):
    //    nu_num = (1/2pi) * arccos( (cos(2*pi*nu_true) + 2) / 3 )
    //  where nu = f / f_waveguide (normalised to waveguide rate).
    idx = 0;
    size_t warped_bins = 0;
    filt.run(begin(signal), end(signal), begin(signal),
             [&](auto /*cplx*/, auto freq_norm) -> std::complex<float> {
                 const size_t cur = idx++;

                 //  freq_norm: normalised to output rate [0, 0.5]
                 const double f = freq_norm * output_sr;
                 const double nu_true = f / waveguide_sr;

                 //  DC bin: no dispersion.
                 if (nu_true < 1e-8) {
                     return (cur < num_bins) ? spectrum[cur]
                                             : std::complex<float>{0, 0};
                 }

                 //  Beyond 49% of waveguide Nyquist: discard (unreliable).
                 if (nu_true >= 0.49) {
                     return std::complex<float>{0, 0};
                 }

                 //  Inverse dispersion warp.
                 const double cos_true = std::cos(2.0 * M_PI * nu_true);
                 const double arg = (cos_true + 2.0) / 3.0;
                 if (arg < -1.0 || arg > 1.0) {
                     return std::complex<float>{0, 0};
                 }
                 const double nu_num =
                         std::acos(arg) / (2.0 * M_PI);

                 //  Map numerical frequency to source bin index.
                 const double src_bin =
                         nu_num * waveguide_sr /
                         output_sr * static_cast<double>(fft_len);
                 const auto lo = static_cast<size_t>(src_bin);
                 if (lo + 1 >= num_bins) {
                     return std::complex<float>{0, 0};
                 }

                 //  Linear interpolation in the complex domain.
                 const auto frac =
                         static_cast<float>(src_bin - static_cast<double>(lo));
                 ++warped_bins;
                 return spectrum[lo] * (1.0f - frac) +
                        spectrum[lo + 1] * frac;
             });

    fprintf(stderr,
            "[dispersion_compensation] warped %zu/%zu bins "
            "(wg_sr=%.0f out_sr=%.0f)\n",
            warped_bins, num_bins, waveguide_sr, output_sr);
    fflush(stderr);
}

}  // namespace waveguide
}  // namespace wayverb
