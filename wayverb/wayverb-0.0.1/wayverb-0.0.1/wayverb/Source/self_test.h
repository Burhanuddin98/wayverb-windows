#pragma once

#include "main_model.h"
#include <string>
#include <vector>

/// Automated self-test that validates wayverb subsystems without running
/// a full simulation.  Each check returns pass/fail + a short message.
class SelfTest final {
public:
    struct Result {
        std::string name;
        bool passed;
        std::string detail;
    };

    /// Run all checks against the currently loaded project.
    static std::vector<Result> run_all(main_model& model);

    /// Format results into a human-readable report string.
    static std::string format_report(const std::vector<Result>& results);

private:
    static Result check_opencl();
    static Result check_scene_geometry(main_model& model);
    static Result check_source_receiver(main_model& model);
    static Result check_gpu_memory(main_model& model);
    static Result check_materials(main_model& model);
    static Result check_raytracer_config(main_model& model);
    static Result check_waveguide_config(main_model& model);
    static Result check_output_directory(main_model& model);
};
