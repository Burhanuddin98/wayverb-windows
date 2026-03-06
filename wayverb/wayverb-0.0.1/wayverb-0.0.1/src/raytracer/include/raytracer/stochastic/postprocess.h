#pragma once

#include "raytracer/stochastic/postprocessing.h"

#include <cstdio>
#include <algorithm>

namespace wayverb {
namespace raytracer {
namespace stochastic {

struct size_functor final {
    template <typename T>
    auto operator()(const T& t) const {
        return t.size();
    }
};

template <size_t Az, size_t El, typename Method>
auto postprocess(const directional_energy_histogram<Az, El>& histogram,
                 const Method& method,
                 double room_volume,
                 const core::environment& environment,
                 double sample_rate) {
    fprintf(stderr, "[stochastic::postprocess] start: Az=%zu El=%zu room_vol=%.1f sr=%.0f\n",
            Az, El, room_volume, sample_rate);
    fflush(stderr);

    const auto& table = histogram.histogram.table;

    const auto max_size = std::accumulate(
            std::begin(table),
            std::end(table),
            size_t{0},
            [&](auto a, const auto& b) {
                if (std::distance(std::begin(b), std::end(b)) == 0) {
                    return a;
                }
                const auto make_size_iterator = [](auto it) {
                    return util::make_mapping_iterator_adapter(std::move(it),
                                                               size_functor{});
                };
                return std::max(
                        a,
                        *std::max_element(make_size_iterator(std::begin(b)),
                                          make_size_iterator(std::end(b))));
            });

    auto max_seconds = max_size / histogram.sample_rate;

    // Safety cap: limit dirac sequence to 10 seconds to prevent massive
    // FFTW allocations that could crash the system.
    constexpr double MAX_POSTPROCESS_SECONDS = 10.0;
    if (max_seconds > MAX_POSTPROCESS_SECONDS) {
        fprintf(stderr, "[stochastic::postprocess] WARNING: capping max_seconds %.2f -> %.2f\n",
                max_seconds, MAX_POSTPROCESS_SECONDS);
        fflush(stderr);
        max_seconds = MAX_POSTPROCESS_SECONDS;
    }

    fprintf(stderr, "[stochastic::postprocess] max_size=%zu max_seconds=%.3f\n",
            max_size, max_seconds);
    fflush(stderr);

    const auto dirac_sequence = generate_dirac_sequence(
            environment.speed_of_sound, room_volume, sample_rate, max_seconds);

    fprintf(stderr, "[stochastic::postprocess] dirac_sequence len=%zu, calling postprocessing\n",
            dirac_sequence.sequence.size());
    fflush(stderr);

    auto result = postprocessing(
            histogram, method, dirac_sequence, environment.acoustic_impedance);

    fprintf(stderr, "[stochastic::postprocess] done, result len=%zu\n", result.size());
    fflush(stderr);

    return result;
}

}  // namespace stochastic
}  // namespace raytracer
}  // namespace wayverb
