#pragma once

#include "raytracer/image_source/fast_pressure_calculator.h"
#include "raytracer/image_source/tree.h"

#include "core/cl/include.h"
#include "core/geo/geometric.h"
#include "core/geo/triangle_vec.h"
#include "core/spatial_division/voxelised_scene_data.h"

#include "utilities/aligned/vector.h"

#include <vector>

namespace wayverb {
namespace raytracer {
namespace image_source {

/// Result of deterministic enumeration: a validated image-source path.
struct deterministic_path final {
    std::vector<cl_uint> triangles;      ///< ordered triangle indices
    glm::vec3 image_source;              ///< final image source position
    util::aligned::vector<reflection_metadata> intersections;
};

using vsd_type =
        core::voxelised_scene_data<cl_float3,
                                   core::surface<core::simulation_bands>>;

/// Enumerate all valid image-source paths up to max_order for arbitrary
/// triangle meshes.  Returns validated paths with acoustic metadata.
///
/// \param source       source position
/// \param receiver     receiver position
/// \param voxelised    scene with geometry + surfaces
/// \param max_order    maximum reflection order (1-3 recommended)
/// \param max_distance prune paths whose image source exceeds this distance
util::aligned::vector<deterministic_path> enumerate_deterministic_paths(
        const glm::vec3& source,
        const glm::vec3& receiver,
        const vsd_type& voxelised,
        size_t max_order,
        double max_distance);

/// Convert deterministic paths into impulses using the same pressure
/// calculator as the ray-traced code path.
util::aligned::vector<impulse<core::simulation_bands>>
deterministic_paths_to_impulses(
        const util::aligned::vector<deterministic_path>& paths,
        const glm::vec3& receiver,
        const vsd_type& voxelised,
        bool flip_phase);

/// Merge deterministic impulses with ray-traced impulses, removing
/// positional duplicates.  Deterministic results take priority.
util::aligned::vector<impulse<core::simulation_bands>> merge_and_deduplicate(
        util::aligned::vector<impulse<core::simulation_bands>>&& raytrace,
        util::aligned::vector<impulse<core::simulation_bands>>&& deterministic);

}  // namespace image_source
}  // namespace raytracer
}  // namespace wayverb
