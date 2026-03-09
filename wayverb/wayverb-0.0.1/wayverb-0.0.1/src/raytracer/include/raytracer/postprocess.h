#pragma once

#include "raytracer/canonical.h"
#include "raytracer/image_source/postprocess.h"
#include "raytracer/stochastic/postprocess.h"

#include "core/sum_ranges.h"

namespace wayverb {
namespace raytracer {

template <typename Histogram, typename Method>
auto postprocess(const simulation_results<Histogram>& input,
                 const Method& method,
                 const glm::vec3& position,
                 double room_volume,
                 const core::environment& environment,
                 double output_sample_rate) {
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

    //  Architecture rebuild: NO crossfade needed.
    //
    //  The stochastic synthesis now has an onset delay (Kuttruff mixing time)
    //  so it produces zero energy during the early reflection period.
    //  The image-source produces discrete specular spikes that naturally
    //  attenuate with each reflection order.
    //
    //  These two signals are temporally separated by design:
    //    - Image-source: t=0 to ~mixing_time (orders 1-4, discrete spikes)
    //    - Stochastic: mixing_time onwards (diffuse tail, fades in smoothly)
    //
    //  A simple additive sum is correct: the image-source spikes sit cleanly
    //  on top of silence (no stochastic noise to bury them), and the stochastic
    //  tail fades in after the last significant early reflection.

    fprintf(stderr,
            "[raytracer::postprocess] additive sum: head=%zu tail=%zu\n",
            head.size(), tail.size());
    fflush(stderr);

    return core::sum_vectors(head, tail);
}

}  // namespace raytracer
}  // namespace wayverb
