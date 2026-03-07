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
#include <cstdio>

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

template <typename LoIt, typename HiIt>
auto crossover_filter(LoIt b_lo,
                      LoIt e_lo,
                      HiIt b_hi,
                      HiIt e_hi,
                      double cutoff,
                      double width) {
    const auto lo_len = std::distance(b_lo, e_lo);
    const auto hi_len = std::distance(b_hi, e_hi);
    auto fft_bins = frequency_domain::best_fft_length(std::max(lo_len, hi_len))
            << 2;
    fprintf(stderr, "[crossover_filter] lo_len=%td hi_len=%td fft_bins=%zu cutoff=%.4f width=%.4f\n",
            lo_len, hi_len, fft_bins, cutoff, width);
    fflush(stderr);
    frequency_domain::filter filt{fft_bins};

    constexpr auto l = 0;

    const auto run_filter = [&](auto b, auto e, auto mag_func) {
        auto ret = std::vector<float>(std::distance(b, e));
        filt.run(b, e, begin(ret), [&](auto cplx, auto freq) {
            return cplx * static_cast<float>(mag_func(freq, cutoff, width, l));
        });
        return ret;
    };

    const auto lo =
            run_filter(b_lo, e_lo, frequency_domain::compute_lopass_magnitude);
    const auto hi =
            run_filter(b_hi, e_hi, frequency_domain::compute_hipass_magnitude);

    return core::sum_vectors(lo, hi);
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

    //  Individual processing.
    fprintf(stderr, "[combined::postprocess] waveguide postprocess...\n"); fflush(stderr);
    const auto waveguide_processed =
            waveguide::postprocess(input.waveguide,
                                   method,
                                   environment.acoustic_impedance,
                                   output_sample_rate);
    fprintf(stderr, "[combined::postprocess] waveguide done, len=%zu\n",
            waveguide_processed.size()); fflush(stderr);

    fprintf(stderr, "[combined::postprocess] raytracer postprocess...\n"); fflush(stderr);
    const auto raytracer_processed = raytracer::postprocess(input.raytracer,
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

    if (input.waveguide.empty()) {
        return raytracer_processed;
    }

    //  DC-block the raytracer output to match waveguide treatment and
    //  prevent subsonic energy leaking through the crossover.
    auto raytracer_blocked = raytracer_processed;
    {
        constexpr auto dc_block_hz = 30.0;
        const auto dc_block = dc_block_hz / output_sample_rate;
        frequency_domain::filter dc_filt{
                frequency_domain::best_fft_length(raytracer_blocked.size()) << 2};
        dc_filt.run(begin(raytracer_blocked),
                    end(raytracer_blocked),
                    begin(raytracer_blocked),
                    [&](auto cplx, auto freq) {
            return cplx * static_cast<float>(
                frequency_domain::compute_hipass_magnitude(
                    freq, dc_block, 0.3, 0));
        });
    }

    const auto cutoff = *std::max_element(make_iterator(begin(input.waveguide)),
                                          make_iterator(end(input.waveguide))) /
                        output_sample_rate;
    const auto width = 0.2;  //  Wider = more natural-sounding
    auto filtered = crossover_filter(begin(waveguide_processed),
                                     end(waveguide_processed),
                                     begin(raytracer_blocked),
                                     end(raytracer_blocked),
                                     cutoff,
                                     width);

    //  Just in case the start has a bit of a dc offset, we do a sneaky window.
    const auto window_length =
            std::min(filtered.size(),
                     static_cast<size_t>(std::floor(
                             distance(source_position, receiver_position) *
                             output_sample_rate / environment.speed_of_sound)));

    if (window_length != 0) {
        const auto window = core::left_hanning(std::floor(window_length));
        std::transform(
                begin(window),
                end(window),
                begin(filtered),
                begin(filtered),
                [](auto envelope, auto signal) { return envelope * signal; });
    }

    //  Fade out the last 5% of the IR to kill any bass crescendo from
    //  FFT circular convolution or crossover phase artifacts.
    {
        const auto fade_len = std::max(
                size_t{256},
                static_cast<size_t>(filtered.size() * 0.05));
        const auto fade_start = filtered.size() > fade_len
                ? filtered.size() - fade_len : 0;
        for (size_t i = fade_start; i < filtered.size(); ++i) {
            const auto t = static_cast<float>(i - fade_start) /
                           static_cast<float>(fade_len);
            //  Raised cosine fade: 0.5*(1+cos(pi*t))
            filtered[i] *= 0.5f * (1.0f + std::cos(3.14159265f * t));
        }
    }

    return filtered;
}

}  // namespace combined
}  // namespace wayverb
