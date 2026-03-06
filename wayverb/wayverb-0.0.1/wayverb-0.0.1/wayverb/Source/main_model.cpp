#include "main_model.h"

#include "UtilityComponents/async_work_queue.h"

#include "combined/model/hrtf.h"

#include "core/cl/common.h"
#include "core/serialize/range.h"
#include "core/serialize/surface.h"

#include "cereal/archives/json.hpp"
#include "cereal/types/memory.hpp"
#include "cereal/types/string.hpp"
#include "cereal/types/tuple.hpp"

#include "glm/glm.hpp"
#include "core/conversions.h"
#include "waveguide/config.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <queue>
#include <set>

// Diagnostic log file — absolute path so it's always findable.
static std::ofstream& diag_log() {
    static std::ofstream f("C:/RoomGUI/wayverb_diag.log",
                           std::ios::out | std::ios::trunc);
    return f;
}

project::project(const std::string& fpath)
        : scene_data_{is_project_file(fpath) ? compute_model_path(fpath)
                                             : fpath}
        , needs_save_{!is_project_file(fpath)} {

    diag_log() << "[project] loading: " << fpath << "\n";
    diag_log().flush();

    //  First make sure default source and receiver are in a sensible position.
    const auto aabb =
            wayverb::core::geo::compute_aabb(get_scene_data().get_vertices());
    const auto c = centre(aabb);

    (*persistent.sources())[0]->set_position(c);
    (*persistent.receivers())[0]->set_position(c);

    //  Set up surfaces.
    const auto& surface_strings = scene_data_.get_scene_data()->get_surfaces();
    diag_log() << "[project] raw surface count: " << surface_strings.size()
               << "\n";
    for (size_t i = 0; i < surface_strings.size(); ++i) {
        diag_log() << "  surface[" << i << "]: " << surface_strings[i] << "\n";
    }
    diag_log().flush();

    *persistent.materials() = wayverb::combined::model::materials_from_names<1>(
            begin(surface_strings), end(surface_strings));

    // Assign realistic default absorption/scattering based on material names.
    // Without this, all materials get 5% absorption (extremely reflective).
    {
        auto& mats = *persistent.materials();
        for (size_t i = 0; i < mats.size(); ++i) {
            const auto name = mats[i]->get_name();
            // Convert to lowercase for matching
            std::string lower;
            lower.reserve(name.size());
            for (char c : name) lower += static_cast<char>(std::tolower(c));

            float abs_val = 0.10f;   // unknown material default
            float scat_val = 0.10f;

            // Match common material keywords
            if (lower.find("carpet") != std::string::npos) {
                abs_val = 0.30f; scat_val = 0.40f;
            } else if (lower.find("curtain") != std::string::npos || lower.find("drape") != std::string::npos) {
                abs_val = 0.50f; scat_val = 0.30f;
            } else if (lower.find("glass") != std::string::npos || lower.find("window") != std::string::npos) {
                abs_val = 0.06f; scat_val = 0.05f;
            } else if (lower.find("concrete") != std::string::npos) {
                abs_val = 0.04f; scat_val = 0.10f;
            } else if (lower.find("brick") != std::string::npos) {
                abs_val = 0.05f; scat_val = 0.15f;
            } else if (lower.find("plywood") != std::string::npos || lower.find("wood") != std::string::npos) {
                abs_val = 0.15f; scat_val = 0.10f;
            } else if (lower.find("plaster") != std::string::npos || lower.find("drywall") != std::string::npos || lower.find("gypsum") != std::string::npos) {
                abs_val = 0.10f; scat_val = 0.10f;
            } else if (lower.find("tile") != std::string::npos || lower.find("ceramic") != std::string::npos || lower.find("marble") != std::string::npos) {
                abs_val = 0.03f; scat_val = 0.05f;
            } else if (lower.find("metal") != std::string::npos || lower.find("steel") != std::string::npos || lower.find("alumin") != std::string::npos) {
                abs_val = 0.04f; scat_val = 0.05f;
            } else if (lower.find("foam") != std::string::npos || lower.find("acoustic") != std::string::npos || lower.find("absorb") != std::string::npos) {
                abs_val = 0.70f; scat_val = 0.30f;
            } else if (lower.find("fabric") != std::string::npos || lower.find("upholster") != std::string::npos) {
                abs_val = 0.40f; scat_val = 0.30f;
            } else if (lower.find("ceiling") != std::string::npos) {
                abs_val = 0.25f; scat_val = 0.15f;
            } else if (lower.find("floor") != std::string::npos) {
                abs_val = 0.15f; scat_val = 0.10f;
            } else if (lower.find("wall") != std::string::npos) {
                abs_val = 0.12f; scat_val = 0.10f;
            }

            *mats[i] = wayverb::combined::model::material{
                    name,
                    wayverb::core::make_surface<wayverb::core::simulation_bands>(
                            abs_val, scat_val)};
            diag_log() << "  [auto-abs] \"" << name << "\" -> abs="
                       << abs_val << " scat=" << scat_val << "\n";
        }
        diag_log().flush();
    }

    const auto& raw_tris = scene_data_.get_scene_data()->get_triangles();
    const auto& raw_verts = scene_data_.get_scene_data()->get_vertices();
    diag_log() << "[project] triangles: " << raw_tris.size()
               << "  vertices: " << raw_verts.size() << "\n";
    diag_log().flush();

    //  If there's a config file, we'll just overwrite the state we just set,
    //  but that's probably fine.
    if (is_project_file(fpath)) {
        const auto config_file = project::compute_config_path(fpath);
        std::ifstream stream{config_file};
        cereal::JSONInputArchive archive{stream};
        archive(persistent);
    }

    //  Auto-partition if the mesh has very few surface groups.
    {
        if (surface_strings.size() <= 2) {
            diag_log() << "[project] auto-partitioning...\n";
            diag_log().flush();
            auto_partition_surfaces();
            diag_log() << "[project] auto-partition done. materials: "
                       << persistent.materials()->size() << "\n";
            for (size_t i = 0; i < persistent.materials()->size(); ++i) {
                diag_log() << "  mat[" << i << "]: "
                           << (*persistent.materials())[i]->get_name() << "\n";
            }
            diag_log().flush();
        } else {
            diag_log() << "[project] skipping auto-partition ("
                       << surface_strings.size() << " surfaces)\n";
            diag_log().flush();
        }
    }

    // Auto-detect scale: if bounding box > 100 in any axis, assume non-meter
    // units and pick the most likely scale factor.
    {
        const auto det_aabb = wayverb::core::geo::compute_aabb(
                scene_data_.get_scene_data()->get_vertices());
        const auto dim = dimensions(det_aabb);
        const auto max_dim = std::max({dim.x, dim.y, dim.z});
        if (max_dim > 1000.0f) {
            scale_factor_ = 0.001f;  // mm → m
            diag_log() << "[project] auto-scale: mm->m (max_dim="
                       << max_dim << ")\n";
        } else if (max_dim > 100.0f) {
            scale_factor_ = 0.01f;   // cm → m
            diag_log() << "[project] auto-scale: cm->m (max_dim="
                       << max_dim << ")\n";
        } else {
            diag_log() << "[project] scale: meters (max_dim="
                       << max_dim << ")\n";
        }
        diag_log().flush();

        // Re-position source/receiver at the scaled centre
        if (scale_factor_ != 1.0f) {
            const auto scaled_data = get_scene_data();
            const auto scaled_aabb = wayverb::core::geo::compute_aabb(
                    scaled_data.get_vertices());
            const auto sc = centre(scaled_aabb);
            (*persistent.sources())[0]->set_position(sc);
            (*persistent.receivers())[0]->set_position(sc);
        }
    }

    // Log material presets count
    diag_log() << "[project] material_presets will be loaded in main_model\n";
    diag_log().flush();

    // === PIPELINE VERIFICATION ===
    // Simulate what generate_scene_data() will do at render time:
    // build material_map from persistent.materials(), call get_scene_data(),
    // then check if all surface names match.
    {
        diag_log() << "\n[VERIFY] === Pipeline Verification at Load Time ===\n";

        // 1. Build material_map (same as generate_scene_data)
        const auto& mats = *persistent.materials();
        diag_log() << "[VERIFY] persistent.materials() (" << mats.size() << "):\n";
        for (size_t i = 0; i < mats.size(); ++i) {
            const auto& surf = mats[i]->get_surface();
            diag_log() << "  mat[" << i << "] name=\"" << mats[i]->get_name()
                       << "\" abs[0]=" << surf.absorption.s[0]
                       << " scat[0]=" << surf.scattering.s[0] << "\n";
        }

        // 2. Get scene data (same as get_scene_data())
        const auto sd = get_scene_data();
        const auto& surfaces = sd.get_surfaces();
        const auto& tris = sd.get_triangles();
        diag_log() << "[VERIFY] get_scene_data() surfaces (" << surfaces.size()
                   << "), triangles (" << tris.size() << "):\n";

        // 3. Check name matching
        size_t found = 0, not_found = 0;
        for (size_t i = 0; i < surfaces.size(); ++i) {
            bool matched = false;
            for (size_t j = 0; j < mats.size(); ++j) {
                if (mats[j]->get_name() == surfaces[i]) {
                    matched = true;
                    break;
                }
            }
            diag_log() << "  surf[" << i << "]=\"" << surfaces[i] << "\" -> "
                       << (matched ? "FOUND" : "** NOT FOUND **") << "\n";
            if (matched) ++found; else ++not_found;
        }

        // 4. Check triangle surface index distribution
        std::map<cl_uint, size_t> surf_usage;
        for (const auto& tri : tris) {
            surf_usage[tri.surface]++;
        }
        diag_log() << "[VERIFY] Triangle-to-surface distribution:\n";
        for (const auto& [idx, count] : surf_usage) {
            diag_log() << "  surface_idx=" << idx << " -> " << count << " tris";
            if (idx < surfaces.size()) {
                diag_log() << " (\"" << surfaces[idx] << "\")";
            } else {
                diag_log() << " ** OUT OF RANGE! **";
            }
            diag_log() << "\n";
        }

        diag_log() << "[VERIFY] triangle_overrides_ has "
                   << triangle_overrides_.size() << " entries\n";
        diag_log() << "[VERIFY] Result: " << found << " matched, "
                   << not_found << " NOT matched\n";
        if (not_found > 0) {
            diag_log() << "[VERIFY] *** WARNING: " << not_found
                       << " surface names have NO matching material! "
                       << "Engine will use ZERO absorption (100% reflective)! ***\n";
        }
        diag_log() << "[VERIFY] === End Pipeline Verification ===\n\n";
        diag_log().flush();
    }

    persistent.connect([&](auto&) { needs_save_ = true; });
}

std::string project::compute_model_path(const std::string& root) {
    return root + '/' + model_name;
}

std::string project::compute_config_path(const std::string& root) {
    return root + '/' + config_name;
}

bool project::is_project_file(const std::string& fpath) {
    return std::string{std::find_if(crbegin(fpath),
                                    crend(fpath),
                                    [](auto i) { return i == '.'; })
                               .base(),
                       end(fpath)} == project_extension;
}

std::string project::get_extensions() const {
    return scene_data_.get_extensions() + ';' + project_extension;
}

void project::save_to(const std::string& fpath) {
    if (needs_save_) {
        //  TODO create directory

        //  write current geometry to file
        scene_data_.save(project::compute_model_path(fpath));

        //  write config with all current materials to file
        std::ofstream stream{project::compute_config_path(fpath)};
        cereal::JSONOutputArchive{stream}(persistent);

        needs_save_ = false;
    }
}

bool project::needs_save() const { return needs_save_; }

void project::set_scale(float s) {
    if (s == scale_factor_) return;
    scale_factor_ = s;
    needs_save_ = true;
    if (on_scale_changed) on_scale_changed();
}

wayverb::core::generic_scene_data<cl_float3, std::string>
project::get_scene_data() const {
    auto data = *scene_data_.get_scene_data();
    auto triangles = data.get_triangles();

    // Apply uniform scale to vertices
    auto vertices = data.get_vertices();
    if (scale_factor_ != 1.0f) {
        for (auto& v : vertices) {
            v.s[0] *= scale_factor_;
            v.s[1] *= scale_factor_;
            v.s[2] *= scale_factor_;
        }
    }

    // When triangle_overrides_ is populated (auto-partition or manual),
    // build the surfaces list entirely from persistent materials so that
    // names match for the engine's material-map lookup.
    const auto& mats = *persistent.materials();

    if (!triangle_overrides_.empty()) {
        // Apply per-triangle surface overrides
        for (size_t i = 0; i < triangles.size(); ++i) {
            auto it = triangle_overrides_.find(i);
            if (it != triangle_overrides_.end()) {
                triangles[i].surface = it->second;
            }
        }

        // Build surfaces list from materials (replaces original names)
        util::aligned::vector<std::string> surfaces;
        surfaces.reserve(mats.size());
        for (size_t i = 0; i < mats.size(); ++i) {
            surfaces.push_back(mats[i]->get_name());
        }

        return wayverb::core::make_scene_data(
                std::move(triangles),
                std::move(vertices),
                std::move(surfaces));
    }

    // No overrides — use original surfaces, extending if materials were added
    auto surfaces = data.get_surfaces();
    if (mats.size() > surfaces.size()) {
        for (size_t i = surfaces.size(); i < mats.size(); ++i) {
            surfaces.push_back(mats[i]->get_name());
        }
    }

    return wayverb::core::make_scene_data(
            std::move(triangles),
            std::move(vertices),
            std::move(surfaces));
}

void project::auto_partition_surfaces() {
    const auto& data = *scene_data_.get_scene_data();
    const auto& triangles = data.get_triangles();
    const auto& vertices = data.get_vertices();
    const auto num_tris = triangles.size();

    if (num_tris == 0) return;

    // 1. Compute face normal for each triangle
    std::vector<glm::vec3> normals(num_tris);
    for (size_t i = 0; i < num_tris; ++i) {
        const auto& tri = triangles[i];
        const auto v0 = wayverb::core::to_vec3{}(vertices[tri.v0]);
        const auto v1 = wayverb::core::to_vec3{}(vertices[tri.v1]);
        const auto v2 = wayverb::core::to_vec3{}(vertices[tri.v2]);
        auto n = glm::cross(v1 - v0, v2 - v0);
        const auto len = glm::length(n);
        normals[i] = len > 1e-12f ? n / len : glm::vec3{0, 1, 0};
    }

    // 2. Weld vertices by position (handles triangle-soup meshes where each
    //    triangle has unique vertex indices but co-located positions).
    //    Map each vertex position to a canonical index.
    diag_log() << "[partition] welding vertices...\n";
    diag_log().flush();

    // Quantize positions to avoid floating-point comparison issues.
    // Use a grid with cell size ~0.0001 (sub-millimetre precision).
    auto quantize = [](float v) -> int64_t {
        return static_cast<int64_t>(std::round(v * 10000.0f));
    };
    struct pos_hash {
        size_t operator()(const std::tuple<int64_t, int64_t, int64_t>& t) const {
            auto h1 = std::hash<int64_t>{}(std::get<0>(t));
            auto h2 = std::hash<int64_t>{}(std::get<1>(t));
            auto h3 = std::hash<int64_t>{}(std::get<2>(t));
            return h1 ^ (h2 * 2654435761ULL) ^ (h3 * 40503ULL);
        }
    };

    std::unordered_map<std::tuple<int64_t, int64_t, int64_t>, cl_uint, pos_hash>
            pos_to_weld;
    pos_to_weld.reserve(vertices.size());

    // For each vertex, find or assign a canonical welded index.
    std::vector<cl_uint> weld_id(vertices.size());
    cl_uint next_weld = 0;
    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto p = wayverb::core::to_vec3{}(vertices[i]);
        auto key = std::make_tuple(quantize(p.x), quantize(p.y), quantize(p.z));
        auto it = pos_to_weld.find(key);
        if (it != pos_to_weld.end()) {
            weld_id[i] = it->second;
        } else {
            weld_id[i] = next_weld;
            pos_to_weld[key] = next_weld++;
        }
    }

    diag_log() << "[partition] welded " << vertices.size()
               << " verts -> " << next_weld << " unique positions\n";
    diag_log().flush();

    // Build adjacency using welded vertex positions.
    // Edge key = ordered pair of welded vertex IDs.
    auto make_edge_hash = [](cl_uint a, cl_uint b) -> uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<uint64_t>(a) << 32) | static_cast<uint64_t>(b);
    };

    std::unordered_map<uint64_t, std::vector<size_t>> edge_to_tris;
    edge_to_tris.reserve(num_tris * 3);

    for (size_t i = 0; i < num_tris; ++i) {
        const auto& tri = triangles[i];
        cl_uint ws[3] = {weld_id[tri.v0], weld_id[tri.v1], weld_id[tri.v2]};
        for (int e = 0; e < 3; ++e) {
            auto key = make_edge_hash(ws[e], ws[(e + 1) % 3]);
            edge_to_tris[key].push_back(i);
        }
    }

    // Build per-triangle neighbour list
    std::vector<std::vector<size_t>> adjacency(num_tris);
    for (const auto& [key, tris] : edge_to_tris) {
        for (size_t a = 0; a < tris.size(); ++a) {
            for (size_t b = a + 1; b < tris.size(); ++b) {
                adjacency[tris[a]].push_back(tris[b]);
                adjacency[tris[b]].push_back(tris[a]);
            }
        }
    }

    diag_log() << "[partition] adjacency built. edges: "
               << edge_to_tris.size() << "\n";
    diag_log().flush();

    // 3. Flood-fill connected components where adjacent triangles have similar
    //    normals (cosine similarity > threshold ≈ 18° tolerance).
    constexpr float cos_threshold = 0.95f;
    std::vector<int> component(num_tris, -1);
    int num_components = 0;

    for (size_t seed = 0; seed < num_tris; ++seed) {
        if (component[seed] >= 0) continue;

        const int comp_id = num_components++;
        std::queue<size_t> bfs;
        bfs.push(seed);
        component[seed] = comp_id;

        while (!bfs.empty()) {
            const auto cur = bfs.front();
            bfs.pop();

            for (const auto neighbour : adjacency[cur]) {
                if (component[neighbour] >= 0) continue;
                // Check normal similarity with the seed normal (not cur)
                // to keep the region coherent.
                if (glm::dot(normals[neighbour], normals[seed]) >=
                    cos_threshold) {
                    component[neighbour] = comp_id;
                    bfs.push(neighbour);
                }
            }
        }
    }

    diag_log() << "[partition] flood-fill done. components: "
               << num_components << "\n";
    diag_log().flush();

    // 4. Compute average normal per component and sort by size (largest first)
    struct region_info {
        int comp_id;
        size_t tri_count;
        glm::vec3 avg_normal;
    };
    std::vector<region_info> regions(num_components);
    for (int i = 0; i < num_components; ++i) {
        regions[i].comp_id = i;
        regions[i].tri_count = 0;
        regions[i].avg_normal = glm::vec3{0};
    }
    for (size_t i = 0; i < num_tris; ++i) {
        auto& r = regions[component[i]];
        r.tri_count++;
        r.avg_normal += normals[i];
    }
    for (auto& r : regions) {
        const auto len = glm::length(r.avg_normal);
        if (len > 1e-12f) r.avg_normal /= len;
    }

    // Sort: largest regions first
    std::sort(regions.begin(), regions.end(),
              [](const auto& a, const auto& b) {
                  return a.tri_count > b.tri_count;
              });

    // Cap at 30 regions — merge tiny fragments into the last bucket ("Other").
    constexpr int max_regions = 30;
    const int kept = std::min(num_components, max_regions);

    diag_log() << "[partition] keeping " << kept << " of "
               << num_components << " regions\n";
    diag_log().flush();

    // Build reverse mapping: old comp_id → sorted index (clamped)
    std::vector<int> comp_to_sorted(num_components);
    for (int i = 0; i < num_components; ++i) {
        // Regions beyond `kept-1` all map to the last bucket
        comp_to_sorted[regions[i].comp_id] = std::min(i, kept - 1);
    }

    // Use `kept` as the effective number of components from here on.
    num_components = kept;

    // 5. Name each region based on its average normal direction.
    //    Y-up convention (standard OBJ).
    std::vector<std::string> region_names(num_components);
    int floor_count = 0, ceiling_count = 0, wall_count = 0;

    for (int i = 0; i < num_components; ++i) {
        const auto& n = regions[i].avg_normal;
        const float ax = std::abs(n.x), ay = std::abs(n.y), az = std::abs(n.z);

        std::string name;
        if (ay >= ax && ay >= az) {
            if (n.y > 0) {
                ceiling_count++;
                name = ceiling_count == 1 ? "Ceiling"
                     : "Ceiling " + std::to_string(ceiling_count);
            } else {
                floor_count++;
                name = floor_count == 1 ? "Floor"
                     : "Floor " + std::to_string(floor_count);
            }
        } else {
            wall_count++;
            name = "Wall " + std::to_string(wall_count);
        }
        region_names[i] = name;
    }

    // If we merged small fragments into the last bucket, rename it.
    if (kept < static_cast<int>(regions.size())) {
        region_names[kept - 1] = "Other";
    }

    // 6. Populate triangle_overrides_ and create material entries.
    triangle_overrides_.clear();

    // Resize materials to match number of regions, assign names and sensible
    // default absorption/scattering based on surface type.
    auto& mats = *persistent.materials();
    mats.resize(static_cast<size_t>(num_components));
    for (int i = 0; i < num_components; ++i) {
        // Pick physically reasonable defaults based on surface classification.
        float abs_val = 0.10f;   // generic wall default
        float scat_val = 0.10f;
        const auto& name = region_names[i];
        if (name.find("Floor") != std::string::npos) {
            abs_val = 0.15f;   // carpet / wood floor
            scat_val = 0.10f;
        } else if (name.find("Ceiling") != std::string::npos) {
            abs_val = 0.25f;   // acoustic ceiling tile
            scat_val = 0.15f;
        } else if (name.find("Wall") != std::string::npos) {
            abs_val = 0.12f;   // painted plaster
            scat_val = 0.10f;
        } else {
            abs_val = 0.10f;   // generic
            scat_val = 0.10f;
        }
        *mats[i] = wayverb::combined::model::material{
                name,
                wayverb::core::make_surface<wayverb::core::simulation_bands>(
                        abs_val, scat_val)};
    }

    // Set triangle overrides
    for (size_t i = 0; i < num_tris; ++i) {
        const auto sorted_idx =
                static_cast<cl_uint>(comp_to_sorted[component[i]]);
        triangle_overrides_[i] = sorted_idx;
    }

    needs_save_ = true;
}

void project::reassign_surface(size_t from_surface, size_t to_surface) {
    if (from_surface == to_surface) return;
    // Reassign all triangles currently mapped to from_surface → to_surface
    const auto& data = *scene_data_.get_scene_data();
    const auto num_tris = data.get_triangles().size();
    for (size_t i = 0; i < num_tris; ++i) {
        cl_uint current;
        auto it = triangle_overrides_.find(i);
        if (it != triangle_overrides_.end()) {
            current = it->second;
        } else {
            current = data.get_triangles()[i].surface;
        }
        if (current == static_cast<cl_uint>(from_surface)) {
            triangle_overrides_[i] = static_cast<cl_uint>(to_surface);
        }
    }
    needs_save_ = true;
    if (on_surfaces_changed) on_surfaces_changed();
}

void project::remove_surface(size_t index) {
    auto& mats = *persistent.materials();
    if (mats.size() <= 1 || index >= mats.size()) return;  // can't delete last

    // Reassign triangles: index → 0, and shift indices above index down by 1
    const auto& data = *scene_data_.get_scene_data();
    const auto num_tris = data.get_triangles().size();
    for (size_t i = 0; i < num_tris; ++i) {
        cl_uint current;
        auto it = triangle_overrides_.find(i);
        if (it != triangle_overrides_.end()) {
            current = it->second;
        } else {
            current = data.get_triangles()[i].surface;
        }
        if (current == static_cast<cl_uint>(index)) {
            triangle_overrides_[i] = 0;  // merge into first surface
        } else if (current > static_cast<cl_uint>(index)) {
            triangle_overrides_[i] = current - 1;  // shift down
        }
    }

    // Remove the material entry
    auto it = mats.begin();
    std::advance(it, index);
    mats.erase(it);

    needs_save_ = true;
    if (on_surfaces_changed) on_surfaces_changed();
}

size_t project::add_surface(const std::string& name) {
    persistent.materials()->push_back(
            wayverb::combined::model::material{name});
    const auto new_index = persistent.materials()->size() - 1;
    needs_save_ = true;
    if (on_surfaces_changed) on_surfaces_changed();
    return new_index;
}

size_t project::get_surface_count() const {
    return persistent.materials()->size();
}

////////////////////////////////////////////////////////////////////////////////

namespace detail {
template <typename T>
class queue_forwarding_call final {
public:
    constexpr queue_forwarding_call(T& t, async_work_queue& queue)
            : t_{t}
            , queue_{queue} {}

    template <typename... Ts>
    void operator()(Ts... ts) const {
        this->queue_.push([ t = &t_, ts... ] { (*t)(ts...); });
    }

private:
    T& t_;
    async_work_queue& queue_;
};

template <typename T>
auto make_queue_forwarding_call(T& t, async_work_queue& queue) {
    return queue_forwarding_call<T>{t, queue};
}

}  // namespace detail

class main_model::impl final {
    template <typename T>
    auto make_queue_forwarding_call(T& t) {
        return detail::make_queue_forwarding_call(t, queue_);
    }

public:
    impl()
            : begun_connection_{engine_.connect_begun(
                      make_queue_forwarding_call(begun_))}
            , engine_state_changed_connection_{engine_.connect_engine_state_changed(
                      make_queue_forwarding_call(engine_state_changed_))}
            , node_positions_changed_connection_{engine_.connect_waveguide_node_positions_changed(
                      make_queue_forwarding_call(node_positions_changed_))}
            , node_pressures_changed_connection_{engine_.connect_waveguide_node_pressures_changed(
                      make_queue_forwarding_call(node_pressures_changed_))}
            , reflections_generated_connection_{engine_.connect_raytracer_reflections_generated(
                      make_queue_forwarding_call(reflections_generated_))}
            , encountered_error_connection_{engine_.connect_encountered_error(
                      make_queue_forwarding_call(encountered_error_))}
            , finished_connection_{engine_.connect_finished(
                      make_queue_forwarding_call(finished_))} {}

    ~impl() noexcept { cancel_render(); }

    void start_render(const class project& project,
                      const wayverb::combined::model::output& output) {
        auto scene_data = generate_scene_data(project);
        auto persistent_copy = project.persistent;

        // Estimate waveguide VRAM usage from bounding box and grid spacing.
        // If it would exceed a safe limit (~4 GB), auto-switch to raytracer-only.
        const auto wg_mode = persistent_copy.waveguide().item()->get_mode();
        if (wg_mode != wayverb::combined::model::waveguide::mode::raytracer_only) {
            constexpr double speed_of_sound = 340.0;
            const auto sample_freq = wayverb::combined::model::compute_sampling_frequency(
                    *persistent_copy.waveguide());
            const auto grid_sp = wayverb::waveguide::config::grid_spacing(
                    speed_of_sound, 1.0 / sample_freq);

            const auto aabb = wayverb::core::geo::compute_aabb(
                    scene_data.get_vertices());
            const auto dim = dimensions(aabb);
            const auto nx = static_cast<size_t>(std::ceil(dim.x / grid_sp)) + 2;
            const auto ny = static_cast<size_t>(std::ceil(dim.y / grid_sp)) + 2;
            const auto nz = static_cast<size_t>(std::ceil(dim.z / grid_sp)) + 2;
            const auto total_nodes = nx * ny * nz;
            // Each node needs ~12 bytes (node type + pressure float + neighbours)
            // plus OpenCL overhead. Rough estimate: 16 bytes per node.
            const auto estimated_vram_mb = total_nodes * 16 / (1024 * 1024);

            fprintf(stderr, "[engine] waveguide grid estimate: %zux%zux%zu = %zu nodes "
                    "(grid_sp=%.3fm, ~%zu MB VRAM)\n",
                    nx, ny, nz, total_nodes, grid_sp, estimated_vram_mb);
            fflush(stderr);

            // 4 GB limit — RTX 2060 has 6 GB but needs headroom for other buffers
            constexpr size_t MAX_VRAM_MB = 4096;
            if (estimated_vram_mb > MAX_VRAM_MB) {
                fprintf(stderr, "[engine] *** Waveguide too large (%zu MB > %zu MB limit). "
                        "Auto-switching to raytracer-only mode. ***\n",
                        estimated_vram_mb, MAX_VRAM_MB);
                fflush(stderr);
                persistent_copy.waveguide().item()->set_mode(
                        wayverb::combined::model::waveguide::mode::raytracer_only);
            }
        }

        engine_.run(wayverb::core::compute_context{},
                    std::move(scene_data),
                    std::move(persistent_copy),
                    output);
    }

    void cancel_render() { engine_.cancel(); }

    bool is_rendering() const { return engine_.is_running(); }

    engine_state_changed::connection connect_engine_state(
            engine_state_changed::callback_type t) {
        return engine_state_changed_.connect(std::move(t));
    }

    waveguide_node_positions_changed::connection connect_node_positions(
            waveguide_node_positions_changed::callback_type t) {
        return node_positions_changed_.connect(std::move(t));
    }

    waveguide_node_pressures_changed::connection connect_node_pressures(
            waveguide_node_pressures_changed::callback_type t) {
        return node_pressures_changed_.connect(std::move(t));
    }

    raytracer_reflections_generated::connection connect_reflections(
            raytracer_reflections_generated::callback_type t) {
        return reflections_generated_.connect(std::move(t));
    }

    encountered_error::connection connect_error_handler(
            encountered_error::callback_type t) {
        return encountered_error_.connect(std::move(t));
    }

    begun::connection connect_begun(begun::callback_type t) {
        return begun_.connect(std::move(t));
    }

    finished::connection connect_finished(finished::callback_type t) {
        return finished_.connect(std::move(t));
    }

private:
    wayverb::core::gpu_scene_data generate_scene_data(
            const class project& project) const {
        util::aligned::unordered_map<
                std::string,
                wayverb::core::surface<wayverb::core::simulation_bands>>
                material_map;

        fprintf(stderr, "[generate_scene_data] building material_map...\n");
        diag_log() << "\n[RENDER] === Material values at render time ===\n";
        for (const auto& i : *project.persistent.materials()) {
            const auto& surf = i->get_surface();
            fprintf(stderr, "  mat name=\"%s\" abs[0]=%.4f scat[0]=%.4f\n",
                    i->get_name().c_str(), surf.absorption.s[0], surf.scattering.s[0]);
            diag_log() << "  mat \"" << i->get_name()
                       << "\" abs=[";
            for (int b = 0; b < 8; ++b)
                diag_log() << (b ? "," : "") << surf.absorption.s[b];
            diag_log() << "] scat=[";
            for (int b = 0; b < 8; ++b)
                diag_log() << (b ? "," : "") << surf.scattering.s[b];
            diag_log() << "]\n";
            material_map[i->get_name()] = surf;
        }

        const auto scene = project.get_scene_data();
        const auto& surfaces = scene.get_surfaces();
        fprintf(stderr, "[generate_scene_data] scene has %zu surfaces, %zu tris\n",
                surfaces.size(), scene.get_triangles().size());
        diag_log() << "[RENDER] scene: " << surfaces.size() << " surfaces, "
                   << scene.get_triangles().size() << " tris\n";
        size_t render_found = 0, render_missing = 0;
        for (size_t i = 0; i < surfaces.size(); ++i) {
            const auto it = material_map.find(surfaces[i]);
            const bool ok = it != material_map.end();
            fprintf(stderr, "  surf[%zu]=\"%s\" -> %s\n", i,
                    surfaces[i].c_str(), ok ? "FOUND" : "NOT_FOUND");
            diag_log() << "  surf[" << i << "]=\"" << surfaces[i]
                       << "\" -> " << (ok ? "FOUND" : "** NOT FOUND **");
            if (ok) {
                diag_log() << " abs[0]=" << it->second.absorption.s[0];
                ++render_found;
            } else {
                diag_log() << " => ZERO ABSORPTION (100% reflective!)";
                ++render_missing;
            }
            diag_log() << "\n";
        }
        diag_log() << "[RENDER] Result: " << render_found << " matched, "
                   << render_missing << " NOT matched\n";
        if (render_missing > 0) {
            diag_log() << "[RENDER] *** WARNING: " << render_missing
                       << " unmatched → engine uses ZERO absorption! ***\n";
        }
        diag_log() << "[RENDER] === End render-time materials ===\n\n";
        diag_log().flush();
        fflush(stderr);

        return scene_with_extracted_surfaces(scene, material_map);
    }

    async_work_queue queue_;

    wayverb::combined::complete_engine engine_;

    begun begun_;
    begun::scoped_connection begun_connection_;

    engine_state_changed engine_state_changed_;
    engine_state_changed::scoped_connection engine_state_changed_connection_;

    waveguide_node_positions_changed node_positions_changed_;
    waveguide_node_positions_changed::scoped_connection
            node_positions_changed_connection_;

    waveguide_node_pressures_changed node_pressures_changed_;
    waveguide_node_pressures_changed::scoped_connection
            node_pressures_changed_connection_;

    raytracer_reflections_generated reflections_generated_;
    raytracer_reflections_generated::scoped_connection
            reflections_generated_connection_;

    encountered_error encountered_error_;
    encountered_error::scoped_connection encountered_error_connection_;

    finished finished_;
    finished::scoped_connection finished_connection_;
};

////////////////////////////////////////////////////////////////////////////////

main_model::main_model(const std::string& name)
        : project{name}
        , material_presets{wayverb::combined::model::presets::materials}
        , capsule_presets{wayverb::combined::model::presets::capsules}
        , currently_open_file_{name}
        , pimpl_{std::make_unique<impl>()} {
    diag_log() << "[main_model] material_presets count: "
               << material_presets.size() << "\n";
    if (!material_presets.empty()) {
        diag_log() << "  first: " << material_presets.front().get_name()
                   << "\n  last:  " << material_presets.back().get_name()
                   << "\n";
    }
    diag_log() << "[main_model] project.materials count: "
               << project.persistent.materials()->size() << "\n";
    diag_log().flush();
}

main_model::~main_model() noexcept = default;

void main_model::start_render() {
    // Ensure output directory is set — default to user home if empty.
    if (output.get_output_directory().empty()) {
        auto home = juce::File::getSpecialLocation(
                juce::File::userHomeDirectory).getFullPathName().toStdString();
        fprintf(stderr, "[main_model] output_directory was empty, defaulting to: %s\n", home.c_str());
        fflush(stderr);
        output.set_output_directory(std::move(home));
    }
    fprintf(stderr, "[main_model] start_render: output_dir='%s'\n",
            output.get_output_directory().c_str());
    fflush(stderr);
    pimpl_->start_render(project, output);
}

void main_model::cancel_render() { pimpl_->cancel_render(); }

bool main_model::is_rendering() const { return pimpl_->is_rendering(); }

void main_model::save(const save_callback& callback) {
    if (project::is_project_file(currently_open_file_)) {
        project.save_to(currently_open_file_);
    } else {
        if (const auto fpath = callback()) {
            save_as(*fpath);
        }
    }
}

void main_model::save_as(std::string name) {
    if (!project::is_project_file(name)) {
        throw std::runtime_error{
                "Save path must have correct project extension."};
    }
    project.save_to(name);
    currently_open_file_ = std::move(name);
}

bool main_model::needs_save() const { return project.needs_save(); }

//  CALLBACKS  /////////////////////////////////////////////////////////////////

main_model::begun::connection main_model::connect_begun(
        begun::callback_type t) {
    return pimpl_->connect_begun(std::move(t));
}

main_model::engine_state_changed::connection main_model::connect_engine_state(
        engine_state_changed::callback_type t) {
    return pimpl_->connect_engine_state(std::move(t));
}

main_model::waveguide_node_positions_changed::connection
main_model::connect_node_positions(
        waveguide_node_positions_changed::callback_type t) {
    return pimpl_->connect_node_positions(std::move(t));
}

main_model::waveguide_node_pressures_changed::connection
main_model::connect_node_pressures(
        waveguide_node_pressures_changed::callback_type t) {
    return pimpl_->connect_node_pressures(std::move(t));
}

main_model::raytracer_reflections_generated::connection
main_model::connect_reflections(
        raytracer_reflections_generated::callback_type t) {
    return pimpl_->connect_reflections(std::move(t));
}

main_model::encountered_error::connection main_model::connect_error_handler(
        encountered_error::callback_type t) {
    return pimpl_->connect_error_handler(std::move(t));
}

main_model::finished::connection main_model::connect_finished(
        finished::callback_type t) {
    return pimpl_->connect_finished(std::move(t));
}

//  MISC FUNCTIONS  ////////////////////////////////////////////////////////////

void main_model::reset_view() {
    //  Set up the scene model so that everything is visible.
    const auto scene_data = project.get_scene_data();
    const auto vertices = util::map_to_vector(begin(scene_data.get_vertices()),
                                              end(scene_data.get_vertices()),
                                              wayverb::core::to_vec3{});

    const auto aabb = wayverb::core::geo::compute_aabb(vertices);
    const auto origin = -centre(aabb);
    const auto radius = glm::distance(aabb.get_min(), aabb.get_max()) / 2;

    scene.set_origin(origin);
    scene.set_eye_distance(2 * radius);
    scene.set_rotation(wayverb::core::az_el{M_PI / 4, M_PI / 6});
}
