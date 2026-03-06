#include "self_test.h"

#include "../JuceLibraryCode/JuceHeader.h"

#include "core/cl/common.h"
#include "core/reverb_time.h"
#include "combined/model/waveguide.h"

#include <cmath>
#include <iomanip>
#include <sstream>

std::vector<SelfTest::Result> SelfTest::run_all(main_model& model) {
    std::vector<Result> results;
    results.push_back(check_opencl());
    results.push_back(check_scene_geometry(model));
    results.push_back(check_source_receiver(model));
    results.push_back(check_gpu_memory(model));
    results.push_back(check_materials(model));
    results.push_back(check_raytracer_config(model));
    results.push_back(check_waveguide_config(model));
    results.push_back(check_output_directory(model));
    return results;
}

std::string SelfTest::format_report(const std::vector<Result>& results) {
    std::ostringstream os;
    int pass = 0, fail = 0;
    for (const auto& r : results) {
        os << (r.passed ? "PASS" : "FAIL") << "  " << r.name;
        if (!r.detail.empty()) {
            os << " -- " << r.detail;
        }
        os << "\n";
        if (r.passed) ++pass; else ++fail;
    }
    os << "\n" << pass << " passed, " << fail << " failed out of "
       << results.size() << " checks.";
    return os.str();
}

SelfTest::Result SelfTest::check_opencl() {
    try {
        wayverb::core::compute_context ctx;
        auto name = ctx.device.getInfo<CL_DEVICE_NAME>();
        auto mem_bytes = ctx.device.getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>();
        auto mem_mb = mem_bytes / (1024 * 1024);
        return {"OpenCL Device", true,
                name + " (" + std::to_string(mem_mb) + " MB)"};
    } catch (const std::exception& e) {
        return {"OpenCL Device", false, e.what()};
    }
}

SelfTest::Result SelfTest::check_scene_geometry(main_model& model) {
    try {
        const auto scene_data = model.project.get_scene_data();
        const auto& tris = scene_data.get_triangles();
        const auto& verts = scene_data.get_vertices();
        if (tris.empty() || verts.empty()) {
            return {"Scene Geometry", false, "No triangles or vertices loaded"};
        }
        const auto volume = wayverb::core::estimate_room_volume(scene_data);
        std::ostringstream detail;
        detail << verts.size() << " vertices, " << tris.size()
               << " triangles, volume ~" << std::fixed
               << std::setprecision(1) << volume << " m^3";
        if (volume < 0.01) {
            return {"Scene Geometry", false,
                    detail.str() + " (volume too small)"};
        }
        return {"Scene Geometry", true, detail.str()};
    } catch (const std::exception& e) {
        return {"Scene Geometry", false, e.what()};
    }
}

SelfTest::Result SelfTest::check_source_receiver(main_model& model) {
    const auto& sources = *model.project.persistent.sources();
    const auto& receivers = *model.project.persistent.receivers();

    if (sources.empty()) {
        return {"Sources & Receivers", false, "No sources defined"};
    }
    if (receivers.empty()) {
        return {"Sources & Receivers", false, "No receivers defined"};
    }

    // Check that source and receiver aren't at the same position
    const auto src_pos = (*sources.begin())->get_position();
    const auto rcv_pos = (*receivers.begin())->get_position();
    const auto dist = glm::distance(src_pos, rcv_pos);

    std::ostringstream detail;
    detail << sources.size() << " source(s), " << receivers.size()
           << " receiver(s), distance=" << std::fixed
           << std::setprecision(2) << dist << " m";

    if (dist < 0.01f) {
        return {"Sources & Receivers", false,
                detail.str() + " (source and receiver overlap!)"};
    }
    return {"Sources & Receivers", true, detail.str()};
}

SelfTest::Result SelfTest::check_gpu_memory(main_model& model) {
    try {
        const auto& wg = *model.project.persistent.waveguide();
        const auto wg_mode = wg.get_mode();

        if (wg_mode == wayverb::combined::model::waveguide::mode::raytracer_only) {
            return {"GPU Memory", true, "Raytracer-only mode (no waveguide)"};
        }

        const auto sr = wayverb::combined::model::compute_sampling_frequency(wg);
        constexpr double speed_of_sound = 340.0;
        const double dx = speed_of_sound / (sr * std::sqrt(3.0));
        const auto volume = wayverb::core::estimate_room_volume(
                model.project.get_scene_data());
        const double est_nodes = volume / (dx * dx * dx);
        constexpr double bytes_per_node = 32.0;
        const double est_mb = est_nodes * bytes_per_node / (1024.0 * 1024.0);

        // Check actual GPU memory
        wayverb::core::compute_context ctx;
        auto gpu_mb = ctx.device.getInfo<CL_DEVICE_GLOBAL_MEM_SIZE>()
                      / (1024 * 1024);

        std::ostringstream detail;
        detail << std::fixed << std::setprecision(0)
               << est_mb << " MB estimated ("
               << std::setprecision(1) << est_nodes / 1e6
               << "M nodes), GPU has " << gpu_mb << " MB";

        if (est_mb > gpu_mb * 0.8) {
            return {"GPU Memory", false,
                    detail.str() + " -- WILL LIKELY CRASH"};
        }
        if (est_mb > gpu_mb * 0.5) {
            return {"GPU Memory", true, detail.str() + " -- tight fit"};
        }
        return {"GPU Memory", true, detail.str()};
    } catch (const std::exception& e) {
        return {"GPU Memory", false, e.what()};
    }
}

SelfTest::Result SelfTest::check_materials(main_model& model) {
    const auto& materials = *model.project.persistent.materials();
    if (materials.empty()) {
        return {"Materials", false, "No materials defined"};
    }
    std::ostringstream detail;
    detail << materials.size() << " material(s) configured";
    return {"Materials", true, detail.str()};
}

SelfTest::Result SelfTest::check_raytracer_config(main_model& model) {
    const auto& rt = *model.project.persistent.raytracer();
    const auto params = rt.get();

    std::ostringstream detail;
    detail << params.rays << " rays, img_src_order="
           << params.maximum_image_source_order;

    if (params.rays == 0) {
        return {"Raytracer Config", false, "Ray count is zero"};
    }
    if (params.rays > 100000) {
        detail << " (very high ray count -- may be slow)";
    }
    return {"Raytracer Config", true, detail.str()};
}

SelfTest::Result SelfTest::check_waveguide_config(main_model& model) {
    const auto& wg = *model.project.persistent.waveguide();
    const auto mode = wg.get_mode();

    std::string mode_str;
    switch (mode) {
        case wayverb::combined::model::waveguide::mode::single:
            mode_str = "single-band"; break;
        case wayverb::combined::model::waveguide::mode::multiple:
            mode_str = "multi-band"; break;
        case wayverb::combined::model::waveguide::mode::raytracer_only:
            mode_str = "raytracer-only"; break;
    }

    if (mode == wayverb::combined::model::waveguide::mode::raytracer_only) {
        return {"Waveguide Config", true, mode_str};
    }

    const auto sr = wayverb::combined::model::compute_sampling_frequency(wg);
    std::ostringstream detail;
    detail << mode_str << ", sampling freq=" << std::fixed
           << std::setprecision(0) << sr << " Hz";
    return {"Waveguide Config", true, detail.str()};
}

SelfTest::Result SelfTest::check_output_directory(main_model& model) {
    const auto dir = model.output.get_output_directory();
    if (dir.empty()) {
        return {"Output Directory", false,
                "Not set (use File > Save to set output location)"};
    }

    // Check if directory exists
    File f(dir);
    if (!f.exists()) {
        return {"Output Directory", false,
                "'" + dir + "' does not exist"};
    }
    if (!f.isDirectory()) {
        return {"Output Directory", false,
                "'" + dir + "' is not a directory"};
    }
    return {"Output Directory", true, dir};
}
