#include "combined/threaded_engine.h"
#include "combined/forwarding_call.h"
#include "combined/model/waveguide.h"
#include "combined/validate_placements.h"
#include "combined/waveguide_base.h"

#include "core/cl/include.h"

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/stat.h>
#endif

#include "waveguide/config.h"

#include "core/dsp_vector_ops.h"
#include "core/environment.h"

#include "waveguide/mesh.h"

#include "audio_file/audio_file.h"

namespace wayverb {
namespace combined {
namespace {

struct max_mag_functor final {
    template <typename T>
    auto operator()(const T& t) const {
        return core::max_mag(t.data);
    }
};

struct channel_info final {
    util::aligned::vector<float> data;
    std::string file_name;
};

}  // namespace

std::unique_ptr<capsule_base> polymorphic_capsule_model(
        const model::capsule& i, const core::orientation& orientation) {
    switch (i.get_mode()) {
        case model::capsule::mode::microphone:
            return make_capsule_ptr(i.microphone().item()->get(), orientation);
        case model::capsule::mode::hrtf:
            return make_capsule_ptr(i.hrtf().item()->get(), orientation);
    }
}

/// Null waveguide for raytracer-only mode — returns empty output instantly.
class null_waveguide final : public waveguide_base {
public:
    std::unique_ptr<waveguide_base> clone() const override {
        return std::make_unique<null_waveguide>();
    }

    double compute_sampling_frequency() const override {
        // Return a reasonable frequency so the waveguide mesh has sensible
        // granularity.  At 1 kHz the grid spacing is ~0.2 m which is coarse
        // enough to be lightweight but fine enough not to confuse placement
        // checks.
        return 1000.0;
    }

    std::optional<util::aligned::vector<waveguide::bandpass_band>>
    run(const core::compute_context&,
        const waveguide::voxels_and_mesh&,
        const glm::vec3&,
        const glm::vec3&,
        const core::environment&,
        double,
        const std::atomic_bool&,
        std::function<void(cl::CommandQueue&,
                           const cl::Buffer&,
                           size_t, size_t)>) override {
        fprintf(stderr, "[engine] raytracer-only mode — skipping waveguide\n");
        fflush(stderr);
        return util::aligned::vector<waveguide::bandpass_band>{};
    }
};

std::unique_ptr<waveguide_base> polymorphic_waveguide_model(
        const model::waveguide& i) {
    switch (i.get_mode()) {
        case model::waveguide::mode::single:
            return make_waveguide_ptr(i.single_band().item()->get());
        case model::waveguide::mode::multiple:
            return make_waveguide_ptr(i.multiple_band().item()->get());
        case model::waveguide::mode::raytracer_only:
            return std::make_unique<null_waveguide>();
    }
}

////////////////////////////////////////////////////////////////////////////////

complete_engine::~complete_engine() noexcept {
    cancel();
    if (future_.valid()) {
        try { future_.get(); } catch (...) {}
    }
}

bool complete_engine::is_running() const { return is_running_; }
void complete_engine::cancel() { keep_going_ = false; }

void complete_engine::run(core::compute_context compute_context,
                          core::gpu_scene_data scene_data,
                          model::persistent persistent,
                          model::output output) {
    cancel();

    // Move the old future into the new async task.  The wait happens on a
    // worker thread, NOT the message thread, avoiding a deadlock when the
    // old render's finished_() callback needs to post to the message thread.
    auto old_future = std::move(future_);

    future_ = std::async(std::launch::async, [
        this,
        old_future = std::move(old_future),
        compute_context = std::move(compute_context),
        scene_data = std::move(scene_data),
        persistent = std::move(persistent),
        output = std::move(output)
    ]() mutable {
        // Wait for the previous render to finish (worker thread — safe).
        if (old_future.valid()) {
            try { old_future.get(); } catch (...) {}
        }

        do_run(std::move(compute_context),
               std::move(scene_data),
               std::move(persistent),
               std::move(output));
    });
}

void complete_engine::do_run(core::compute_context compute_context,
                             core::gpu_scene_data scene_data,
                             model::persistent persistent,
                             model::output output) {
    try {
        is_running_ = true;
        keep_going_ = true;

        fprintf(stderr, "[engine] do_run: started\n"); fflush(stderr);

        //  Send the "IT HAS BEGUN" message.
        begun_();

        constexpr core::environment environment{};

        fprintf(stderr, "[engine] do_run: speed_of_sound=%.1f\n",
                environment.speed_of_sound); fflush(stderr);

        //  First, we check that all the sources and receivers are valid, to
        //  avoid doing useless work.

        const auto make_position_extractor_iterator = [](auto it) {
            return util::make_mapping_iterator_adapter(
                    std::move(it),
                    [](const auto& i) { return i.item()->get_position(); });
        };

        const auto is_raytracer_only =
                persistent.waveguide()->get_mode() ==
                model::waveguide::mode::raytracer_only;

        const auto grid_spacing = waveguide::config::grid_spacing(
                environment.speed_of_sound,
                1 / compute_sampling_frequency(*persistent.waveguide()));
        fprintf(stderr, "[engine] do_run: waveguide grid_spacing=%.4f m  raytracer_only=%d\n",
                grid_spacing, int(is_raytracer_only)); fflush(stderr);

        // Skip pairwise distance check in raytracer-only mode since
        // there is no waveguide grid to worry about.
        if (!is_raytracer_only &&
            !is_pairwise_distance_acceptable(
                    make_position_extractor_iterator(
                            std::begin(*persistent.sources())),
                    make_position_extractor_iterator(
                            std::end(*persistent.sources())),
                    make_position_extractor_iterator(
                            std::begin(*persistent.receivers())),
                    make_position_extractor_iterator(
                            std::end(*persistent.receivers())),
                    grid_spacing)) {
            throw std::runtime_error{
                    "Placing sources and receivers too close "
                    "together will produce inaccurate results."};
        }

        {
            //  Check that all sources and receivers are inside the mesh.
            fprintf(stderr, "[engine] do_run: building voxelised scene for placement check\n"); fflush(stderr);
            const auto voxelised =
                    core::make_voxelised_scene_data(scene_data, 5, 0.1f);
            fprintf(stderr, "[engine] do_run: voxelised scene built OK\n"); fflush(stderr);

            if (!are_all_inside(make_position_extractor_iterator(
                                        std::begin(*persistent.sources())),
                                make_position_extractor_iterator(
                                        std::end(*persistent.sources())),
                                voxelised)) {
                throw std::runtime_error{"Source is outside mesh."};
            }

            if (!are_all_inside(make_position_extractor_iterator(
                                        std::begin(*persistent.receivers())),
                                make_position_extractor_iterator(
                                        std::end(*persistent.receivers())),
                                voxelised)) {
                throw std::runtime_error{"Receiver is outside mesh."};
            }
            fprintf(stderr, "[engine] do_run: source/receiver placement OK\n"); fflush(stderr);
        }

        //  Now we can start rendering.

        const auto poly_waveguide =
                polymorphic_waveguide_model(*persistent.waveguide().item());

        std::vector<channel_info> all_channels;

        const auto runs = persistent.sources().item()->size() *
                          persistent.receivers().item()->size();

        auto run = 0;

        //  For each source-receiver pair.
        for (auto source = std::begin(*persistent.sources().item()),
                  e_source = std::end(*persistent.sources().item());
             source != e_source && keep_going_;
             ++source) {
            for (auto receiver = std::begin(*persistent.receivers().item()),
                      e_receiver = std::end(*persistent.receivers().item());
                 receiver != e_receiver && keep_going_;
                 ++receiver, ++run) {
                fprintf(stderr, "[engine] do_run: building postprocessing_engine (source/receiver %d)\n", run); fflush(stderr);
                //  Set up an engine to use.
                postprocessing_engine eng{compute_context,
                                          scene_data,
                                          source->item()->get_position(),
                                          receiver->item()->get_position(),
                                          environment,
                                          persistent.raytracer().item()->get(),
                                          poly_waveguide->clone(),
                                          source->item()->get_directivity(),
                                          source->item()->get_orientation()};

                fprintf(stderr, "[engine] do_run: postprocessing_engine created OK\n"); fflush(stderr);

                //  Send new node position notification.
                waveguide_node_positions_changed_(
                        eng.get_voxels_and_mesh().mesh.get_descriptor());

                //  Register callbacks.
                if (!engine_state_changed_.empty()) {
                    eng.connect_engine_state_changed([this, runs, run](
                            auto state, auto progress) {
                        engine_state_changed_(run, runs, state, progress);
                    });
                }

                if (!waveguide_node_pressures_changed_.empty()) {
                    eng.connect_waveguide_node_pressures_changed(
                            make_forwarding_call(
                                    waveguide_node_pressures_changed_));
                }

                if (!raytracer_reflections_generated_.empty()) {
                    eng.connect_raytracer_reflections_generated(
                            make_forwarding_call(
                                    raytracer_reflections_generated_));
                }

                const auto polymorphic_capsules = util::map_to_vector(
                        std::begin(*receiver->item()->capsules().item()),
                        std::end(*receiver->item()->capsules().item()),
                        [&](const auto& i) {
                            return polymorphic_capsule_model(
                                    *i.item(),
                                    receiver->item()->get_orientation());
                        });

                //  Run the simulation, cache the result.
                fprintf(stderr, "[engine] do_run: calling eng.run() sample_rate=%.0f\n",
                        (double)get_sample_rate(output.get_sample_rate())); fflush(stderr);
                auto channel =
                        eng.run(begin(polymorphic_capsules),
                                end(polymorphic_capsules),
                                get_sample_rate(output.get_sample_rate()),
                                keep_going_);
                fprintf(stderr, "[engine] do_run: eng.run() completed, channel=%s\n",
                        channel ? "OK" : "NULL"); fflush(stderr);

                //  If user cancelled while processing the channel, channel
                //  will be null, but we want to exit before throwing an
                //  exception.
                if (!keep_going_) {
                    break;
                }

                if (!channel) {
                    throw std::runtime_error{
                            "Encountered unknown error, causing channel not to "
                            "be rendered."};
                }

                for (size_t i = 0,
                            e = receiver->item()->capsules().item()->size();
                     i != e;
                     ++i) {
                    all_channels.emplace_back(channel_info{
                            std::move((*channel)[i]),
                            compute_output_path(
                                    *source->item(),
                                    *receiver->item(),
                                    *(*receiver->item()->capsules().item())[i]
                                             .item(),
                                    output)});
                }
            }
        }

        //  If keep going is false now, then the simulation was cancelled.
        if (keep_going_) {
            if (all_channels.empty()) {
                throw std::runtime_error{"No channels were rendered."};
            }

            fprintf(stderr, "[engine] applying ceiling limiter to %zu channels...\n",
                    all_channels.size());
            fflush(stderr);

            //  Ceiling limiter at -1 dBTP (0.891).
            //  Only scales DOWN if the peak exceeds the ceiling — never
            //  amplifies.  This preserves the distance-based normalization
            //  (direct sound = 1/d) while preventing clipping in integer
            //  output formats.
            const auto make_iterator = [](auto it) {
                return util::make_mapping_iterator_adapter(std::move(it),
                                                           max_mag_functor{});
            };

            const auto max_mag =
                    *std::max_element(make_iterator(begin(all_channels)),
                                      make_iterator(end(all_channels)));

            if (max_mag == 0.0f) {
                throw std::runtime_error{"All channels are silent."};
            }

            constexpr float ceiling = 0.891f;  //  -1 dBTP
            const auto factor = (max_mag > ceiling)
                    ? static_cast<double>(ceiling) / max_mag
                    : 1.0;

            fprintf(stderr, "[engine] peak=%.4f ceiling=%.3f factor=%.4f%s\n",
                    max_mag, ceiling, factor,
                    factor < 1.0 ? " (limiting)" : " (no change)");
            fflush(stderr);

            if (factor < 1.0) {
                for (auto& channel : all_channels) {
                    for (auto& sample : channel.data) {
                        sample *= factor;
                    }
                }
            }

            //  Write out files.
            for (const auto& i : all_channels) {
                fprintf(stderr, "[engine] writing: %s (%zu samples)\n",
                        i.file_name.c_str(), i.data.size());
                fflush(stderr);

                // Ensure the output directory exists before writing.
                {
                    auto slash = i.file_name.find_last_of("/\\");
                    if (slash != std::string::npos) {
                        auto dir = i.file_name.substr(0, slash);
                        if (!dir.empty()) {
#ifdef _WIN32
                            // CreateDirectoryA is fine — path already ASCII/locale.
                            CreateDirectoryA(dir.c_str(), nullptr);
#else
                            ::mkdir(dir.c_str(), 0755);
#endif
                        }
                    }
                }

                try {
                    audio_file::write(i.file_name.c_str(),
                                      i.data,
                                      get_sample_rate(output.get_sample_rate()),
                                      output.get_format(),
                                      output.get_bit_depth());
                    fprintf(stderr, "[engine] wrote OK: %s\n",
                            i.file_name.c_str());
                    fflush(stderr);
                } catch (const std::exception& e) {
                    fprintf(stderr, "[engine] WRITE FAILED: %s — %s\n",
                            i.file_name.c_str(), e.what());
                    fflush(stderr);
                    throw;
                }
            }

            fprintf(stderr, "[engine] all files written successfully\n");
            fflush(stderr);
        }

    } catch (const cl::Error& e) {
        char buf[256];
        snprintf(buf, sizeof(buf), "%s (CL error %d)", e.what(), e.err());
        fprintf(stderr, "[engine] cl::Error: %s\n", buf);
        fflush(stderr);
        encountered_error_(buf);
    } catch (const std::exception& e) {
        fprintf(stderr, "[engine] std::exception: %s\n", e.what());
        fflush(stderr);
        encountered_error_(e.what());
    } catch (...) {
        fprintf(stderr, "[engine] UNKNOWN exception caught!\n");
        fflush(stderr);
        encountered_error_("Unknown fatal error during simulation.");
    }

    is_running_ = false;

    finished_();
}

complete_engine::engine_state_changed::connection
complete_engine::connect_engine_state_changed(
        engine_state_changed::callback_type callback) {
    return engine_state_changed_.connect(std::move(callback));
}

complete_engine::waveguide_node_positions_changed::connection
complete_engine::connect_waveguide_node_positions_changed(
        waveguide_node_positions_changed::callback_type callback) {
    return waveguide_node_positions_changed_.connect(std::move(callback));
}

complete_engine::waveguide_node_pressures_changed::connection
complete_engine::connect_waveguide_node_pressures_changed(
        waveguide_node_pressures_changed::callback_type callback) {
    return waveguide_node_pressures_changed_.connect(std::move(callback));
}

complete_engine::raytracer_reflections_generated::connection
complete_engine::connect_raytracer_reflections_generated(
        raytracer_reflections_generated::callback_type callback) {
    return raytracer_reflections_generated_.connect(std::move(callback));
}

complete_engine::encountered_error::connection
complete_engine::connect_encountered_error(
        encountered_error::callback_type callback) {
    return encountered_error_.connect(std::move(callback));
}

complete_engine::begun::connection complete_engine::connect_begun(
        begun::callback_type callback) {
    return begun_.connect(std::move(callback));
}

complete_engine::finished::connection complete_engine::connect_finished(
        finished::callback_type callback) {
    return finished_.connect(std::move(callback));
}

}  // namespace combined
}  // namespace wayverb
