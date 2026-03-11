#include "waveguide/mesh.h"
#include "waveguide/boundary_adjust.h"
#include "waveguide/config.h"
#include "waveguide/fitted_boundary.h"
#include "waveguide/mesh_setup_program.h"
#include "waveguide/program.h"

#include "core/conversions.h"
#include "core/scene_data_loader.h"
#include "core/spatial_division/scene_buffers.h"
#include "core/spatial_division/voxelised_scene_data.h"

#include "utilities/popcount.h"

#include <algorithm>
#include <cstdio>
#include <iostream>

namespace wayverb {
namespace waveguide {

mesh::mesh(mesh_descriptor descriptor, vectors vectors)
        : descriptor_(std::move(descriptor))
        , vectors_(std::move(vectors)) {}

const mesh_descriptor& mesh::get_descriptor() const { return descriptor_; }
const vectors& mesh::get_structure() const { return vectors_; }

bool is_inside(const mesh& m, size_t node_index) {
    return is_inside(m.get_structure().get_condensed_nodes()[node_index]);
}

void mesh::set_coefficients(coefficients_canonical coefficients) {
    vectors_.set_coefficients(coefficients);
}

void mesh::set_coefficients(
        util::aligned::vector<coefficients_canonical> coefficients) {
    vectors_.set_coefficients(std::move(coefficients));
}

double estimate_volume(const mesh& mesh) {
    const auto& nodes = mesh.get_structure().get_condensed_nodes();
    const auto num_inside =
            std::count_if(cbegin(nodes), cend(nodes), [](const auto& i) {
                return is_inside(i);
            });
    const auto spacing = mesh.get_descriptor().spacing;
    const auto node_volume = spacing * spacing * spacing;
    return node_volume * num_inside;
}

////////////////////////////////////////////////////////////////////////////////

mesh compute_mesh(
        const core::compute_context& cc,
        const core::voxelised_scene_data<cl_float3,
                                         core::surface<core::simulation_bands>>&
                voxelised,
        float mesh_spacing,
        float speed_of_sound) {
    const auto program = setup_program{cc};
    auto queue = cl::CommandQueue{cc.context, cc.device};

    const auto buffers = make_scene_buffers(cc.context, voxelised);

    const auto desc = [&] {
        const auto aabb = voxelised.get_voxels().get_aabb();
        const auto dim = glm::ivec3{dimensions(aabb) / mesh_spacing};
        return mesh_descriptor{core::to_cl_float3{}(aabb.get_min()),
                               core::to_cl_int3{}(dim),
                               mesh_spacing};
    }();

    auto nodes = [&] {
        const auto num_nodes = compute_num_nodes(desc);

        fprintf(stderr, "[mesh] num_nodes=%zu\n", num_nodes);
        fflush(stderr);

        cl::Buffer node_buffer{cc.context,
                               CL_MEM_READ_WRITE,
                               num_nodes * sizeof(condensed_node)};

        //  Batch size to keep each GPU dispatch under ~1 second (TDR safe).
        //  set_node_inside fires 32 rays per node, so 128K nodes per batch
        //  is conservative for an RTX 2060.
        constexpr size_t BATCH_SIZE = 128 * 1024;

        //  find whether each node is inside or outside the model
        //  Batched dispatch to avoid GPU TDR timeout on Windows.
        {
            auto kernel = program.get_node_inside_kernel();
            for (size_t offset = 0; offset < num_nodes; offset += BATCH_SIZE) {
                const auto count = std::min(BATCH_SIZE, num_nodes - offset);
                kernel(cl::EnqueueArgs(queue,
                                       cl::NDRange(offset),   // global_offset
                                       cl::NDRange(count),    // global_size
                                       cl::NullRange),        // local_size (auto)
                       node_buffer,
                       desc,
                       buffers.get_voxel_index_buffer(),
                       buffers.get_global_aabb(),
                       buffers.get_side(),
                       buffers.get_triangles_buffer(),
                       buffers.get_vertices_buffer());
                queue.finish();  // yield to OS between batches
                fprintf(stderr, "[mesh] set_node_inside: %zu / %zu\n",
                        std::min(offset + count, num_nodes), num_nodes);
                fflush(stderr);
            }
        }

#ifndef NDEBUG
        {
            auto nodes =
                    core::read_from_buffer<condensed_node>(queue, node_buffer);
            const auto count =
                    count_boundary_type(nodes.begin(), nodes.end(), [](auto i) {
                        return i == id_inside;
                    });
            if (!count) {
                throw std::runtime_error{"No inside nodes found."};
            }
        }
#endif

        //  find node boundary type (lighter kernel, but batch anyway)
        {
            auto kernel = program.get_node_boundary_kernel();
            for (size_t offset = 0; offset < num_nodes; offset += BATCH_SIZE) {
                const auto count = std::min(BATCH_SIZE, num_nodes - offset);
                kernel(cl::EnqueueArgs(queue,
                                       cl::NDRange(offset),
                                       cl::NDRange(count),
                                       cl::NullRange),
                       node_buffer, desc);
                queue.finish();
            }
        }

        fprintf(stderr, "[mesh] mesh setup complete\n"); fflush(stderr);

        return core::read_from_buffer<condensed_node>(queue, node_buffer);
    }();

    //  IMPORTANT
    //  compute_boundary_index_data mutates the nodes array, so it must
    //  be run before condensing the nodes.
    auto boundary_data =
            compute_boundary_index_data(cc.device, buffers, desc, nodes);

    auto v = vectors{
            std::move(nodes),
            util::map_to_vector(
                    begin(voxelised.get_scene_data().get_surfaces()),
                    end(voxelised.get_scene_data().get_surfaces()),
                    [&](const auto& surface) -> coefficients_canonical {
                        try {
                            //  Clamp absorption to a minimum of 0.05 per band.
                            //  Very low absorption (e.g. glass, α ≈ 0.018)
                            //  produces reflectance ≈ 0.991 → IIR filter poles
                            //  at |z| ≈ 0.999, which accumulate numerical error
                            //  over thousands of time steps and diverge to inf.
                            //  A 5% floor is standard practice in commercial
                            //  room-acoustics software (Odeon, CATT-Acoustic).
                            constexpr float min_absorption = 0.05f;
                            core::bands_type clamped_abs;
                            for (int i = 0; i < core::simulation_bands; ++i) {
                                clamped_abs.s[i] = std::max(
                                        surface.absorption.s[i], min_absorption);
                            }
                            return to_impedance_coefficients(
                                    compute_reflectance_filter_coefficients(
                                            clamped_abs.s,
                                            1 / config::time_step(speed_of_sound,
                                                                  mesh_spacing)));
                        } catch (const std::exception& e) {
                            //  Filter design failed (extreme spectral shape).
                            //  Fall back to flat coefficient using mean absorption.
                            fprintf(stderr,
                                    "[mesh] WARNING: filter design failed (%s), "
                                    "using mean absorption fallback\n",
                                    e.what());
                            fflush(stderr);
                            double mean_abs = 0;
                            for (int i = 0; i < core::simulation_bands; ++i) {
                                mean_abs += surface.absorption.s[i];
                            }
                            mean_abs /= core::simulation_bands;
                            return to_flat_coefficients(mean_abs);
                        }
                    }),
            std::move(boundary_data)};

    return {desc, std::move(v)};
}

voxels_and_mesh compute_voxels_and_mesh(const core::compute_context& cc,
                                        const core::gpu_scene_data& scene,
                                        const glm::vec3& anchor,
                                        double sample_rate,
                                        double speed_of_sound) {
    const auto mesh_spacing =
            config::grid_spacing(speed_of_sound, 1 / sample_rate);
    auto voxelised = make_voxelised_scene_data(
            scene,
            5,
            waveguide::compute_adjusted_boundary(
                    core::geo::compute_aabb(scene.get_vertices()),
                    anchor,
                    mesh_spacing));
    auto mesh = compute_mesh(cc, voxelised, mesh_spacing, speed_of_sound);
    return {std::move(voxelised), std::move(mesh)};
}

}  // namespace waveguide
}  // namespace wayverb
