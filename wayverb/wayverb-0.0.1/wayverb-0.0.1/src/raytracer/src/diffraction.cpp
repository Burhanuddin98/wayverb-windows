#include "raytracer/diffraction.h"

#include "core/cl/triangle.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <unordered_map>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace wayverb {
namespace raytracer {
namespace diffraction {

namespace {

/// Hash for an undirected edge (pair of vertex indices).
struct edge_key {
    size_t v0, v1;
    bool operator==(const edge_key& o) const {
        return v0 == o.v0 && v1 == o.v1;
    }
};

struct edge_key_hash {
    size_t operator()(const edge_key& k) const {
        auto h0 = std::hash<size_t>{}(k.v0);
        auto h1 = std::hash<size_t>{}(k.v1);
        return h0 ^ (h1 * 2654435761u);
    }
};

edge_key make_edge_key(size_t a, size_t b) {
    return a < b ? edge_key{a, b} : edge_key{b, a};
}

/// Compute the exterior wedge angle between two faces sharing an edge.
/// Returns angle in radians in (0, 2*pi).  For a convex edge (like a room
/// corner viewed from inside), the exterior angle is > pi.
float compute_wedge_angle(const glm::vec3& n0, const glm::vec3& n1) {
    const auto cos_a = glm::dot(n0, n1);
    //  Interior dihedral = acos(cos_a).
    //  Exterior wedge angle = 2*pi - interior = 2*pi - acos(cos_a).
    //  But for UTD we use n*pi where n = exterior/pi.
    //  Simpler: exterior = pi + acos(dot(n0, n1)) for convex edges.
    //  For concave edges: exterior = pi - acos(dot(n0, n1)).
    //
    //  General formula using cross product to determine convexity:
    //  We define the exterior wedge angle as the angle traversed going
    //  from one face to the other around the OUTSIDE of the wedge.
    const auto interior = std::acos(std::clamp(static_cast<double>(cos_a),
                                               -1.0, 1.0));
    //  For room acoustics, edges are typically convex (interior corners).
    //  The exterior wedge angle = 2*pi - interior_dihedral.
    return static_cast<float>(2.0 * M_PI - interior);
}

/// Fresnel integral approximation (Boersma, 1960).
/// Returns F(x) = C(x) + j*S(x) where C and S are Fresnel cosine/sine integrals.
/// For UTD we need the transition function F(x) = 2j*sqrt(x)*exp(jx)*fresnel_int.
std::complex<double> utd_transition_function(double x) {
    if (x < 0.0) x = 0.0;
    if (x < 0.8) {
        //  Small argument: F(x) ≈ sqrt(pi*x) * exp(j*pi/4) * (1 - j*sqrt(pi*x)*...)
        //  Simplified: for small x, F(x) → 0 (shadow boundary).
        const auto sqrtx = std::sqrt(x);
        return std::complex<double>{sqrtx, sqrtx} *
               std::complex<double>{0.5, 0.0};
    }
    //  Large argument: F(x) → 1 (deep lit region).
    //  Asymptotic expansion: F(x) ≈ 1 + j/(2x) - 3/(4x²) - ...
    if (x > 10.0) {
        return {1.0, 0.0};
    }
    //  Intermediate: rational approximation.
    const auto sqrtx = std::sqrt(x);
    const auto phase = std::exp(std::complex<double>{0.0, x + M_PI / 4.0});
    const auto f_mag = 1.0 - 0.5 / (x + 0.5);
    return phase * f_mag / (sqrtx + 1e-10);
}

/// Closest point on line segment to minimize total distance from source + receiver.
/// This finds the diffraction point on the edge via Fermat's principle.
glm::vec3 find_diffraction_point(const glm::vec3& v0,
                                  const glm::vec3& v1,
                                  const glm::vec3& source,
                                  const glm::vec3& receiver) {
    //  Parametric: P(t) = v0 + t*(v1-v0), t in [0,1].
    //  Minimize: f(t) = |source - P(t)| + |receiver - P(t)|.
    //  Use golden section search (robust, no derivatives needed).
    const auto edge = v1 - v0;
    const auto edge_len = glm::length(edge);
    if (edge_len < 1e-6f) return v0;

    auto eval = [&](float t) -> float {
        const auto p = v0 + t * edge;
        return glm::length(source - p) + glm::length(receiver - p);
    };

    //  Golden section search on [0, 1].
    const float phi = (std::sqrt(5.0f) - 1.0f) / 2.0f;
    float a = 0.0f, b = 1.0f;
    float c = b - phi * (b - a);
    float d = a + phi * (b - a);
    for (int iter = 0; iter < 50; ++iter) {
        if (eval(c) < eval(d)) {
            b = d;
        } else {
            a = c;
        }
        c = b - phi * (b - a);
        d = a + phi * (b - a);
        if (b - a < 1e-6f) break;
    }
    return v0 + 0.5f * (a + b) * edge;
}

}  // namespace

util::aligned::vector<mesh_edge> detect_edges(
        const core::triangle* triangles,
        size_t num_triangles,
        const glm::vec3* vertices,
        size_t num_vertices,
        float max_dihedral_deg) {
    //  Build edge → triangle adjacency map.
    //  Each edge (pair of vertex indices) maps to the triangles sharing it.
    struct tri_info {
        size_t triangle_index;
        glm::vec3 normal;
    };
    std::unordered_map<edge_key, std::vector<tri_info>, edge_key_hash>
            edge_map;

    for (size_t t = 0; t < num_triangles; ++t) {
        const auto& tri = triangles[t];
        const auto& va = vertices[tri.v0];
        const auto& vb = vertices[tri.v1];
        const auto& vc = vertices[tri.v2];
        const auto normal = glm::normalize(glm::cross(vb - va, vc - va));

        const size_t verts[3] = {
                static_cast<size_t>(tri.v0),
                static_cast<size_t>(tri.v1),
                static_cast<size_t>(tri.v2)};
        for (int i = 0; i < 3; ++i) {
            const auto key = make_edge_key(verts[i], verts[(i + 1) % 3]);
            edge_map[key].push_back({t, normal});
        }
    }

    //  Find edges shared by exactly 2 triangles with significant dihedral angle.
    const float max_dihedral_rad =
            max_dihedral_deg * static_cast<float>(M_PI) / 180.0f;

    util::aligned::vector<mesh_edge> result;
    for (const auto& [key, tris] : edge_map) {
        if (tris.size() != 2) continue;

        const auto& n0 = tris[0].normal;
        const auto& n1 = tris[1].normal;
        const auto cos_dihedral = glm::dot(n0, n1);
        const auto dihedral =
                std::acos(std::clamp(cos_dihedral, -1.0f, 1.0f));

        if (dihedral > max_dihedral_rad) continue;  //  Nearly coplanar, skip.

        const auto wedge = compute_wedge_angle(n0, n1);

        result.push_back({vertices[key.v0], vertices[key.v1], n0, n1, wedge});
    }

    fprintf(stderr,
            "[diffraction] detected %zu significant edges from %zu triangles\n",
            result.size(), num_triangles);
    fflush(stderr);

    return result;
}

util::aligned::vector<impulse<core::simulation_bands>>
compute_diffraction_impulses(
        const util::aligned::vector<mesh_edge>& edges,
        const glm::vec3& source,
        const glm::vec3& receiver,
        const core::environment& environment,
        double sample_rate) {
    util::aligned::vector<impulse<core::simulation_bands>> result;

    //  Band center frequencies (Hz) for the 16 simulation bands.
    //  Geometric spacing from ~50 Hz to ~20 kHz.
    constexpr int NB = core::simulation_bands;
    const double f_min = 50.0;
    const double f_max = 20000.0;
    double fc[NB];
    for (int i = 0; i < NB; ++i) {
        fc[i] = f_min * std::pow(f_max / f_min,
                                  (i + 0.5) / static_cast<double>(NB));
    }

    const double c = environment.speed_of_sound;

    for (const auto& edge : edges) {
        //  Find the diffraction point on this edge.
        const auto diff_pt =
                find_diffraction_point(edge.v0, edge.v1, source, receiver);

        const auto d_s = static_cast<double>(glm::length(source - diff_pt));
        const auto d_r = static_cast<double>(glm::length(receiver - diff_pt));

        if (d_s < 1e-4 || d_r < 1e-4) continue;

        const auto total_distance = d_s + d_r;
        const auto delay = total_distance / c;

        //  Geometric spreading factor for diffraction.
        //  For a straight edge: 1 / sqrt(d_s * d_r * (d_s + d_r))
        const auto spread = 1.0 / std::sqrt(d_s * d_r * (d_s + d_r));

        //  Wedge parameter n = wedge_angle / pi.
        const auto n = static_cast<double>(edge.wedge_angle) / M_PI;

        //  Compute per-band UTD diffraction coefficient.
        //  |D| ∝ 1/sqrt(k) = 1/sqrt(2*pi*f/c) → strongest at low frequencies.
        impulse<NB> imp{};
        imp.position = {diff_pt.x, diff_pt.y, diff_pt.z};
        imp.distance = static_cast<float>(total_distance);

        for (int b = 0; b < NB; ++b) {
            const auto k = 2.0 * M_PI * fc[b] / c;  //  Wavenumber.

            //  Simplified UTD coefficient magnitude for a wedge:
            //  |D| = (1 / (n * sqrt(2*pi*k))) * sum of cotangent terms
            //  For a 90-degree corner (n=1.5): |D| ≈ 0.5 / sqrt(k)
            //  For a general wedge:
            const auto D_mag =
                    1.0 / (n * std::sqrt(2.0 * M_PI * k)) *
                    //  Sum of 4 cotangent terms (simplified — using the
                    //  dominant term for the lit/shadow boundary).
                    //  Full Kouyoumjian-Pathak has transition functions F()
                    //  near shadow boundaries; we use the asymptotic form
                    //  which is accurate away from boundaries.
                    std::abs(1.0 / std::tan(M_PI / (2.0 * n)));

            //  Diffraction amplitude = D * spread * source_energy.
            //  Assume unit source energy; the raytracer will apply actual
            //  source weighting.
            const auto amp = static_cast<float>(D_mag * spread);

            imp.volume.s[b] = amp;
        }

        result.push_back(imp);
    }

    fprintf(stderr,
            "[diffraction] computed %zu diffraction impulses from %zu edges\n",
            result.size(), edges.size());
    fflush(stderr);

    return result;
}

}  // namespace diffraction
}  // namespace raytracer
}  // namespace wayverb
