#pragma once

#include "raytracer/canonical.h"
#include "raytracer/image_source/postprocess.h"
#include "raytracer/stochastic/postprocess.h"

#include "core/pressure_intensity.h"
#include "core/sinc.h"
#include "core/sum_ranges.h"

namespace wayverb {
namespace raytracer {

/// Carries the postprocessed time-domain signal together with metadata
/// about the direct sound (needed by the combined engine for normalization).
struct postprocessed_result final {
    util::aligned::vector<float> signal;
    float direct_amplitude;   ///< 0 if LOS was blocked
    float direct_distance;    ///< source-receiver distance
};

/// Place a single broadband impulse into a time-domain signal as a
/// Blackman-windowed sinc spike — identical maths to sinc_sum_functor but
/// operating on a scalar (post-mixdown) buffer.
inline void place_direct_spike(util::aligned::vector<float>& signal,
                               double sample_rate,
                               float distance,
                               double speed_of_sound,
                               float amplitude) {
    const auto direct_sample = (static_cast<double>(distance) / speed_of_sound)
                               * sample_rate;
    const auto centre = static_cast<size_t>(direct_sample);
    constexpr int HALF_W = 8;

    if (centre + HALF_W + 1 >= signal.size()) {
        signal.resize(centre + HALF_W + 2, 0.0f);
    }

    const auto frac = direct_sample - static_cast<double>(centre);

    for (int k = -HALF_W; k <= HALF_W; ++k) {
        const auto idx = static_cast<ptrdiff_t>(centre) + k;
        if (idx < 0 || static_cast<size_t>(idx) >= signal.size()) continue;

        const auto t = static_cast<double>(k) - frac;
        const auto offset = (t / (2 * HALF_W)) + 0.5;
        const auto envelope =
                0.42 - 0.5 * std::cos(2 * M_PI * offset)
                     + 0.08 * std::cos(4 * M_PI * offset);
        const auto s = core::sinc(t) * envelope;
        signal[idx] += static_cast<float>(amplitude * s);
    }

    // Measure actual peak and correct to target amplitude.
    float actual_peak = 0.0f;
    for (int k = -HALF_W; k <= HALF_W; ++k) {
        const auto idx = static_cast<ptrdiff_t>(centre) + k;
        if (idx < 0 || static_cast<size_t>(idx) >= signal.size()) continue;
        actual_peak = std::max(actual_peak, std::abs(signal[idx]));
    }
    if (actual_peak > 1e-10f && std::abs(actual_peak - amplitude) > amplitude * 0.01f) {
        const auto correction = amplitude / actual_peak;
        for (int k = -HALF_W; k <= HALF_W; ++k) {
            const auto idx = static_cast<ptrdiff_t>(centre) + k;
            if (idx < 0 || static_cast<size_t>(idx) >= signal.size()) continue;
            signal[idx] *= correction;
        }
    }
}

template <typename Histogram, typename Method>
auto postprocess(const simulation_results<Histogram>& input,
                 const Method& method,
                 const glm::vec3& position,
                 double room_volume,
                 const core::environment& environment,
                 double output_sample_rate) {
    //  Reflections go through the multiband filterbank (frequency-dependent
    //  absorption per surface) as before.
    auto head =
            raytracer::image_source::postprocess(begin(input.image_source),
                                                 end(input.image_source),
                                                 method,
                                                 position,
                                                 environment.speed_of_sound,
                                                 output_sample_rate);

    auto tail = raytracer::stochastic::postprocess(input.stochastic,
                                                    method,
                                                    room_volume,
                                                    environment,
                                                    output_sample_rate);

    fprintf(stderr,
            "[raytracer::postprocess] additive sum: head=%zu tail=%zu\n",
            head.size(), tail.size());
    fflush(stderr);

    auto signal = core::sum_vectors(head, tail);

    //  Direct sound bypasses the filterbank entirely — place it as a clean
    //  broadband sinc spike at the correct sample position.
    float direct_amp = 0.0f;
    float direct_dist = 0.0f;
    if (input.direct) {
        const auto& d = *input.direct;
        //  The impulse volume is an N-band vector with distance attenuation
        //  already applied.  For the direct (flat-spectrum) path, all bands
        //  are equal — take band 0 as the scalar amplitude.
        direct_amp = d.volume.s[0];
        direct_dist = d.distance;

        place_direct_spike(signal, output_sample_rate,
                           direct_dist, environment.speed_of_sound,
                           direct_amp);

        fprintf(stderr,
                "[raytracer::postprocess] direct sound: d=%.2fm amp=%.6f "
                "sample=%zu\n",
                direct_dist, direct_amp,
                static_cast<size_t>(
                        static_cast<double>(direct_dist) /
                        environment.speed_of_sound * output_sample_rate));
        fflush(stderr);
    } else {
        fprintf(stderr, "[raytracer::postprocess] no LOS — direct sound omitted\n");
        fflush(stderr);
    }

    return postprocessed_result{std::move(signal), direct_amp, direct_dist};
}

}  // namespace raytracer
}  // namespace wayverb
