#include "raytracer/reflection_processor/image_source.h"
#include "raytracer/image_source/deterministic.h"
#include "raytracer/image_source/get_direct.h"
#include "raytracer/image_source/postprocess_branches.h"

#include "core/pressure_intensity.h"

#include <iostream>

namespace wayverb {
namespace raytracer {
namespace reflection_processor {

image_source_group_processor::image_source_group_processor(size_t max_order,
                                                           size_t items)
        : max_image_source_order_{max_order}
        , builder_{items} {}

////////////////////////////////////////////////////////////////////////////////

image_source_processor::image_source_processor(
        const glm::vec3& source,
        const glm::vec3& receiver,
        const core::environment& environment,
        const core::voxelised_scene_data<cl_float3,
                                         core::surface<core::simulation_bands>>&
                voxelised,
        size_t max_order,
        size_t deterministic_order)
        : source_{source}
        , receiver_{receiver}
        , environment_{environment}
        , voxelised_{voxelised}
        , max_order_{max_order}
        , deterministic_order_{deterministic_order} {}

image_source_group_processor image_source_processor::get_group_processor(
        size_t num_directions) const {
    return {max_order_, num_directions};
}

void image_source_processor::accumulate(
        const image_source_group_processor& processor) {
    for (const auto& path : processor.get_results()) {
        tree_.push(path);
    }
}

util::aligned::vector<impulse<core::simulation_bands>> image_source_processor::get_results() const {
    //  Fetch the stochastic (ray-traced) image source results.
    auto ret = raytracer::image_source::postprocess_branches(
            begin(tree_.get_branches()),
            end(tree_.get_branches()),
            source_,
            receiver_,
            voxelised_,
            false);

    //  Deterministic image-source enumeration for low-order reflections.
    if (deterministic_order_ > 0) {
        //  Use scene diagonal as max distance for pruning.
        const auto aabb = voxelised_.get_voxels().get_aabb();
        const auto diagonal = glm::distance(aabb.get_min(), aabb.get_max());
        //  Allow image sources up to 3x the room diagonal (higher-order
        //  reflections can place images far outside the room).
        const double max_dist = diagonal * 3.0;

        auto det_paths = image_source::enumerate_deterministic_paths(
                source_, receiver_, voxelised_, deterministic_order_,
                max_dist);
        auto det_impulses = image_source::deterministic_paths_to_impulses(
                det_paths, receiver_, voxelised_, false);

        ret = image_source::merge_and_deduplicate(
                std::move(ret), std::move(det_impulses));
    }

    //  Add the line-of-sight contribution, which isn't directly detected by
    //  the image-source machinery.
    using namespace image_source;
    if (const auto direct = get_direct(source_, receiver_, voxelised_)) {
        ret.emplace_back(*direct);
    }

    //  Correct for distance travelled.
    for (auto& imp : ret) {
        imp.volume *= core::pressure_for_distance(
                imp.distance, environment_.acoustic_impedance);
    }

    return ret;
}

////////////////////////////////////////////////////////////////////////////////

make_image_source::make_image_source(size_t max_order,
                                     size_t deterministic_order)
        : max_order_{max_order}
        , deterministic_order_{deterministic_order} {}

image_source_processor make_image_source::get_processor(
        const core::compute_context& /*cc*/,
        const glm::vec3& source,
        const glm::vec3& receiver,
        const core::environment& environment,
        const core::voxelised_scene_data<cl_float3,
                                         core::surface<core::simulation_bands>>&
                voxelised) const {
    return {source, receiver, environment, voxelised, max_order_,
            deterministic_order_};
}

}  // namespace reflection_processor
}  // namespace raytracer
}  // namespace wayverb
