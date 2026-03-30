#pragma once

/// \file diffraction.h
/// First-order edge diffraction using the Uniform Theory of Diffraction (UTD).
/// Kouyoumjian & Pathak, "A uniform geometrical theory of diffraction for an
/// edge in a perfectly conducting surface," Proc. IEEE 62(11), 1974.
///
/// For each significant edge in the mesh (dihedral angle < 170 degrees),
/// computes the diffracted impulse from source via edge to receiver.
/// The diffraction coefficient D(f) scales as 1/sqrt(f), so diffraction
/// is strongest at low frequencies.

#include "raytracer/cl/structs.h"

#include "core/cl/scene_structs.h"
#include "core/cl/triangle.h"
#include "core/environment.h"

#include "utilities/aligned/vector.h"

#include "glm/glm.hpp"

namespace wayverb {
namespace raytracer {
namespace diffraction {

/// An edge shared by two triangles, with its wedge angle.
struct mesh_edge final {
    glm::vec3 v0;       ///< Edge start vertex.
    glm::vec3 v1;       ///< Edge end vertex.
    glm::vec3 n0;       ///< Normal of face 0.
    glm::vec3 n1;       ///< Normal of face 1.
    float wedge_angle;  ///< Exterior wedge angle in radians (pi < angle < 2*pi).
};

/// Detect significant diffraction edges from scene geometry.
/// Returns edges where the dihedral angle is less than max_dihedral_deg.
util::aligned::vector<mesh_edge> detect_edges(
        const core::triangle* triangles,
        size_t num_triangles,
        const glm::vec3* vertices,
        size_t num_vertices,
        float max_dihedral_deg = 170.0f);

/// Compute first-order UTD diffraction impulses for all detected edges.
/// Returns one impulse per edge (if the diffraction point exists).
util::aligned::vector<impulse<core::simulation_bands>>
compute_diffraction_impulses(
        const util::aligned::vector<mesh_edge>& edges,
        const glm::vec3& source,
        const glm::vec3& receiver,
        const core::environment& environment,
        double sample_rate);

}  // namespace diffraction
}  // namespace raytracer
}  // namespace wayverb
