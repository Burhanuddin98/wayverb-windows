#pragma once

#include "waveguide/attenuator.h"
#include "waveguide/canonical.h"
#include "waveguide/config.h"

#include "core/attenuator/hrtf.h"
#include "core/attenuator/microphone.h"
#include "core/cl/iterator.h"
#include "core/cl/scene_structs.h"
#include "core/mixdown.h"

#include "utilities/map_to_vector.h"

namespace wayverb {
namespace waveguide {

template <typename T>
using dereferenced_t = decltype(*std::declval<T>());

template <typename T, typename U>
constexpr auto dereferenced_type_matches_v =
        std::is_same<std::decay_t<dereferenced_t<T>>, U>::value;

/// We need a unified interface for dealing with single-band microphone output
/// and multi-band hrtf output.
/// We try to determine whether the iterator is over a single- or multi-band
/// type, and then filter appropriately in the multi-band case.

/// We start with the 'audible range' defined in Hz.
/// This gives us well-defined 8-band edges and widths, also in Hz.

/// If the iterator is over `bands_type` use this one.
/// Audible range is normalised in terms of the waveguide sampling rate.
template <typename It,
          std::enable_if_t<std::is_same<std::decay_t<dereferenced_t<It>>,
                                        core::bands_type>::value,
                           int> = 0>
auto postprocess(It begin, It end, double sample_rate) {
    return core::multiband_filter_and_mixdown(
            begin, end, sample_rate, [](auto it, auto index) {
                return core::make_cl_type_iterator(std::move(it), index);
            });
}

/// If the iterator is over a floating-point type use this one.
template <typename It,
          std::enable_if_t<std::is_floating_point<
                                   std::decay_t<dereferenced_t<It>>>::value,
                           int> = 0>
auto postprocess(It begin, It end, double sample_rate) {
    return util::aligned::vector<float>(begin, end);
}

////////////////////////////////////////////////////////////////////////////////

template <typename Method>
auto postprocess(const band& band,
                 const Method& method,
                 double acoustic_impedance,
                 double output_sample_rate) {
    auto attenuated = util::map_to_vector(
            begin(band.directional),
            end(band.directional),
            make_attenuate_mapper(method, acoustic_impedance));
    const auto ret =
            postprocess(begin(attenuated), end(attenuated), band.sample_rate);

    return waveguide::adjust_sampling_rate(ret.data(),
                                           ret.size(),
                                           band.sample_rate,
                                           output_sample_rate);
}

/// Multiband waveguide postprocessing with unified FFT.
///
/// Each waveguide band has a different native sample rate and produces a
/// different-length signal after resampling.  The old code created a
/// separate FFT filter per band, each with a different FFT size.  This
/// caused misaligned frequency grids: adjacent bands had different bin
/// spacing, leading to phase inconsistencies at band edges when summed.
///
/// The fix: pre-process all bands, find the maximum signal length, and
/// create a SINGLE filter instance with a unified FFT size.  All bands
/// are bandpass-filtered on the same frequency grid, eliminating phase
/// smear at band boundaries.
template <typename Method>
auto postprocess(const util::aligned::vector<bandpass_band>& results,
                 const Method& method,
                 double acoustic_impedance,
                 double output_sample_rate) {
    util::aligned::vector<float> ret;
    if (results.empty()) return ret;

    //  Phase 1: attenuate, mixdown, and resample every band.
    struct band_data {
        util::aligned::vector<float> samples;
        decltype(results[0].valid_hz / output_sample_rate) cutoff;
    };
    std::vector<band_data> all_bands;
    all_bands.reserve(results.size());
    size_t max_len = 0;

    for (const auto& band : results) {
        auto processed = postprocess(
                band.band, method, acoustic_impedance, output_sample_rate);
        max_len = std::max(max_len, processed.size());
        all_bands.push_back(
                {std::move(processed), band.valid_hz / output_sample_rate});
    }

    //  Phase 2: single FFT size for all bands — unified frequency grid.
    const auto fft_len =
            frequency_domain::best_fft_length(max_len) << 3;
    frequency_domain::filter filt{fft_len};

    //  l=1 gives steeper band edges (less inter-band phase interaction).
    //  width=0.2 gives smoother blending between adjacent bands.
    constexpr auto l = 1;
    constexpr auto width = 0.2;

    for (auto& bd : all_bands) {
        const auto b = begin(bd.samples);
        const auto e = end(bd.samples);
        filt.run(b, e, b, [&](auto cplx, auto freq) {
            return cplx * static_cast<float>(
                                  frequency_domain::compute_bandpass_magnitude(
                                          freq, bd.cutoff, width, l));
        });

        ret.resize(std::max(ret.size(), bd.samples.size()), 0.0f);
        std::transform(b, e, begin(ret), begin(ret), std::plus<>{});
    }

    //  DC blocking is now handled in combined::postprocess to avoid
    //  redundant/inconsistent filtering stages.

    return ret;
}

}  // namespace waveguide
}  // namespace wayverb
