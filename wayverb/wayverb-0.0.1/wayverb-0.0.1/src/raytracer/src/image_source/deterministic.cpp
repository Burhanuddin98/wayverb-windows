#include "raytracer/image_source/deterministic.h"

#include "core/geo/geometric.h"
#include "core/geo/triangle_vec.h"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace wayverb {
namespace raytracer {
namespace image_source {

namespace {

/// Construct a ray from 'from' towards 'to'.  Throws if coincident.
core::geo::ray construct_ray(const glm::vec3& from, const glm::vec3& to) {
    if (from == to) {
        throw std::runtime_error(
                "Tried to construct a ray pointing towards its starting "
                "location.");
    }
    return {from, glm::normalize(to - from)};
}

/// Get the geometric triangle (3 vec3 vertices) for a given index.
auto get_triangle(const vsd_type& voxelised, cl_uint tri_idx) {
    const auto& scene = voxelised.get_scene_data();
    return core::geo::get_triangle_vec3(scene.get_triangles()[tri_idx],
                                        scene.get_vertices().data());
}

/// Mirror a point across a triangle's plane.
glm::vec3 mirror_across(const glm::vec3& p,
                         const vsd_type& voxelised,
                         cl_uint tri_idx) {
    return core::geo::mirror(p, get_triangle(voxelised, tri_idx));
}

/// Validate an image-source path defined by a sequence of triangle indices.
/// Returns the validated path (image source position + reflection metadata)
/// or nullopt if the path is geometrically invalid.
///
/// This replicates the logic from traversal_callback::find_valid_path
/// in tree.cpp, but takes an explicit triangle-index vector instead of
/// the tree's state vector.
std::optional<deterministic_path> validate_path(
        const glm::vec3& source,
        const glm::vec3& receiver,
        const vsd_type& voxelised,
        const std::vector<cl_uint>& triangles) {
    // Build the chain of image sources by mirroring across each triangle.
    std::vector<glm::vec3> image_sources;
    image_sources.reserve(triangles.size());
    glm::vec3 current = source;
    for (const auto tri_idx : triangles) {
        current = mirror_across(current, voxelised, tri_idx);
        image_sources.push_back(current);
    }

    const auto final_image_source = image_sources.back();

    // Degenerate: image source coincides with receiver.
    if (receiver == final_image_source) {
        return std::nullopt;
    }

    // Backward trace: from receiver through each reflection point to source.
    // Walk the triangle list in reverse (matching tree.cpp logic).
    struct {
        glm::vec3 prev_intersection;
        cl_uint prev_surface;
    } accumulator{receiver, ~cl_uint{0}};

    util::aligned::vector<reflection_metadata> intersections;
    intersections.reserve(triangles.size());

    for (int i = static_cast<int>(triangles.size()) - 1; i >= 0; --i) {
        const auto tri_idx = triangles[static_cast<size_t>(i)];
        const auto& img = image_sources[static_cast<size_t>(i)];

        // Ray from previous intersection towards this image source.
        const auto ray = construct_ray(accumulator.prev_intersection, img);

        // Find intersection with the scene (ignore previous surface).
        const auto intersection =
                intersects(voxelised, ray, accumulator.prev_surface);

        // Must hit the expected triangle.
        if (!intersection || intersection->index != tri_idx) {
            return std::nullopt;
        }

        // Compute angle between ray and triangle normal.
        const auto cos_angle = std::min(
                1.0f,
                std::max(0.0f,
                         std::abs(glm::dot(
                                 ray.get_direction(),
                                 core::geo::normal(
                                         get_triangle(voxelised, tri_idx))))));

        const auto surface_index =
                voxelised.get_scene_data().get_triangles()[tri_idx].surface;
        intersections.emplace_back(
                reflection_metadata{surface_index, cos_angle});

        // Advance accumulator to this intersection point.
        accumulator.prev_intersection =
                ray.get_position() +
                ray.get_direction() * intersection->inter.t;
        accumulator.prev_surface = tri_idx;
    }

    // Final check: line-of-sight from source to first reflection point.
    {
        const auto ray =
                construct_ray(source, accumulator.prev_intersection);
        const auto intersection = intersects(voxelised, ray);
        if (!intersection ||
            intersection->index != accumulator.prev_surface) {
            return std::nullopt;
        }
    }

    return deterministic_path{triangles,
                              final_image_source,
                              std::move(intersections)};
}

/// Recursive depth-first enumeration of all image-source paths.
void recurse_enumerate(
        const glm::vec3& source,
        const glm::vec3& receiver,
        const vsd_type& voxelised,
        size_t max_order,
        double max_distance,
        size_t num_triangles,
        glm::vec3 current_image,
        size_t depth,
        std::vector<cl_uint>& path_so_far,
        util::aligned::vector<deterministic_path>& results) {
    for (cl_uint tri = 0; tri < static_cast<cl_uint>(num_triangles); ++tri) {
        // Skip self-reflection (same triangle twice in a row).
        if (!path_so_far.empty() && path_so_far.back() == tri) {
            continue;
        }

        // Mirror current image source across this triangle.
        const auto img = mirror_across(current_image, voxelised, tri);

        // Distance pruning.
        if (glm::distance(img, receiver) > static_cast<float>(max_distance)) {
            continue;
        }

        path_so_far.push_back(tri);

        // Validate this path.
        if (auto valid = validate_path(source, receiver, voxelised,
                                       path_so_far)) {
            results.emplace_back(std::move(*valid));
        }

        // Recurse deeper if not at max order.
        if (depth + 1 < max_order) {
            recurse_enumerate(source, receiver, voxelised, max_order,
                              max_distance, num_triangles, img, depth + 1,
                              path_so_far, results);
        }

        path_so_far.pop_back();
    }
}

}  // namespace

////////////////////////////////////////////////////////////////////////////////

util::aligned::vector<deterministic_path> enumerate_deterministic_paths(
        const glm::vec3& source,
        const glm::vec3& receiver,
        const vsd_type& voxelised,
        size_t max_order,
        double max_distance) {
    if (max_order == 0) {
        return {};
    }

    const auto num_triangles =
            voxelised.get_scene_data().get_triangles().size();

    // Safety: auto-reduce order for large meshes.
    size_t effective_order = max_order;
    {
        double candidates = 1.0;
        for (size_t i = 0; i < max_order; ++i) {
            candidates *= static_cast<double>(num_triangles);
        }
        if (candidates > 10e6) {
            effective_order = std::max(size_t{1}, max_order - 1);
            std::cerr << "[deterministic IS] " << num_triangles
                      << " triangles at order " << max_order
                      << " would produce " << candidates
                      << " candidates; reducing to order "
                      << effective_order << "\n";
        }
    }

    util::aligned::vector<deterministic_path> results;
    std::vector<cl_uint> path_so_far;
    path_so_far.reserve(effective_order);

    recurse_enumerate(source, receiver, voxelised, effective_order,
                      max_distance, num_triangles, source, 0, path_so_far,
                      results);

    std::cerr << "[deterministic IS] found " << results.size()
              << " valid paths (order <= " << effective_order
              << ", " << num_triangles << " triangles)\n";

    return results;
}

////////////////////////////////////////////////////////////////////////////////

util::aligned::vector<impulse<core::simulation_bands>>
deterministic_paths_to_impulses(
        const util::aligned::vector<deterministic_path>& paths,
        const glm::vec3& receiver,
        const vsd_type& voxelised,
        bool flip_phase) {
    const auto& surfaces = voxelised.get_scene_data().get_surfaces();
    auto calculator = make_fast_pressure_calculator(
            surfaces.cbegin(), surfaces.cend(), receiver, flip_phase);

    util::aligned::vector<impulse<core::simulation_bands>> ret;
    ret.reserve(paths.size());

    for (const auto& p : paths) {
        ret.emplace_back(calculator(p.image_source,
                                    p.intersections.cbegin(),
                                    p.intersections.cend()));
    }

    return ret;
}

////////////////////////////////////////////////////////////////////////////////

util::aligned::vector<impulse<core::simulation_bands>> merge_and_deduplicate(
        util::aligned::vector<impulse<core::simulation_bands>>&& raytrace,
        util::aligned::vector<impulse<core::simulation_bands>>&& deterministic) {
    // Position-based deduplication: if a ray-traced impulse has the same
    // image-source position (within tolerance) as a deterministic one,
    // remove the ray-traced version — the deterministic one is exact.
    constexpr float tol = 1e-3f;

    auto is_dup = [&](const impulse<core::simulation_bands>& rt) {
        const glm::vec3 pa{rt.position.s[0], rt.position.s[1],
                           rt.position.s[2]};
        for (const auto& d : deterministic) {
            const glm::vec3 pb{d.position.s[0], d.position.s[1],
                               d.position.s[2]};
            if (glm::distance(pa, pb) < tol) {
                return true;
            }
        }
        return false;
    };

    // Remove ray-traced duplicates.
    raytrace.erase(
            std::remove_if(raytrace.begin(), raytrace.end(), is_dup),
            raytrace.end());

    // Append deterministic results.
    raytrace.insert(raytrace.end(), deterministic.begin(),
                    deterministic.end());

    return std::move(raytrace);
}

}  // namespace image_source
}  // namespace raytracer
}  // namespace wayverb
