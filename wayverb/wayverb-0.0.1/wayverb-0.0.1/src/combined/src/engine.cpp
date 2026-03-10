#include "combined/engine.h"
#include "combined/model/source.h"
#include "combined/postprocess.h"
#include "combined/waveguide_base.h"

#include "waveguide/mesh.h"

#include "raytracer/canonical.h"

#include "core/cl/common.h"
#include "core/cl/scene_structs.h"
#include "core/environment.h"
#include "core/orientation.h"
#include "core/reverb_time.h"
#include "core/scene_data.h"

#include "glm/glm.hpp"

namespace wayverb {
namespace combined {

namespace {
template <typename Histogram>
class intermediate_impl final : public intermediate {
public:
    intermediate_impl(combined_results<Histogram> to_process,
                      const glm::vec3& source_position,
                      const glm::vec3& receiver_position,
                      double room_volume,
                      const core::environment& environment)
            : to_process_{std::move(to_process)}
            , source_position_{source_position}
            , receiver_position_{receiver_position}
            , room_volume_{room_volume}
            , environment_{environment} {}

    util::aligned::vector<float> postprocess(
            const core::attenuator::null& a,
            double sample_rate) const override {
        return postprocess_impl(a, sample_rate);
    }

    util::aligned::vector<float> postprocess(
            const core::attenuator::hrtf& a,
            double sample_rate) const override {
        return postprocess_impl(a, sample_rate);
    }

    util::aligned::vector<float> postprocess(
            const core::attenuator::microphone& a,
            double sample_rate) const override {
        return postprocess_impl(a, sample_rate);
    }

private:
    template <typename Attenuator>
    auto postprocess_impl(const Attenuator& attenuator,
                          double output_sample_rate) const {
        return wayverb::combined::postprocess(to_process_,
                                              attenuator,
                                              source_position_,
                                              receiver_position_,
                                              room_volume_,
                                              environment_,
                                              output_sample_rate);
    }

    combined_results<Histogram> to_process_;
    glm::vec3 source_position_;
    glm::vec3 receiver_position_;
    double room_volume_;
    core::environment environment_;
    engine::engine_state_changed engine_state_changed_;
};

template <typename Histogram>
auto make_intermediate_impl_ptr(combined_results<Histogram> to_process,
                                const glm::vec3& source_position,
                                const glm::vec3& receiver_position,
                                double room_volume,
                                const core::environment& environment) {
    return std::make_unique<intermediate_impl<Histogram>>(std::move(to_process),
                                                          source_position,
                                                          receiver_position,
                                                          room_volume,
                                                          environment);
}

}  // namespace

class engine::impl final {
public:
    impl(const core::compute_context& compute_context,
         const core::gpu_scene_data& scene_data,
         const glm::vec3& source,
         const glm::vec3& receiver,
         const core::environment& environment,
         const raytracer::simulation_parameters& raytracer,
         std::unique_ptr<waveguide_base> waveguide,
         model::directivity_pattern source_directivity,
         const core::orientation& source_orientation)
            : compute_context_{compute_context}
            , voxels_and_mesh_{waveguide::compute_voxels_and_mesh(
                      compute_context,
                      scene_data,
                      receiver,
                      waveguide->compute_sampling_frequency(),
                      environment.speed_of_sound)}
            , room_volume_{estimate_volume(voxels_and_mesh_.mesh)}
            , source_{source}
            , receiver_{receiver}
            , environment_{environment}
            , raytracer_{raytracer}
            , waveguide_{std::move(waveguide)}
            , source_directivity_{source_directivity}
            , source_orientation_{source_orientation} {
        //  Compute Sabine RT60 from scene geometry for physics-based
        //  waveguide duration.  This replaces the arbitrary 1.5x multiplier.
        try {
            const auto absorption =
                    core::equivalent_absorption_area(scene_data);
            core::bands_type air_coeff{};
            const auto rt60 = core::sabine_reverb_time(
                    room_volume_, absorption, air_coeff);
            //  Take the maximum RT60 across all frequency bands.
            //  Low-frequency bands typically have the longest decay.
            double max_rt = 0.0;
            for (int i = 0; i < core::simulation_bands; ++i) {
                max_rt = std::max(max_rt, static_cast<double>(rt60.s[i]));
            }
            sabine_rt60_max_ = max_rt;
            fprintf(stderr,
                    "[engine] Sabine RT60 max=%.2f s (across %d bands)\n",
                    sabine_rt60_max_, core::simulation_bands);
            fflush(stderr);
        } catch (const std::exception& e) {
            fprintf(stderr,
                    "[engine] WARNING: Sabine RT60 failed: %s, using "
                    "default %.1f s\n",
                    e.what(), sabine_rt60_max_);
            fflush(stderr);
        }
    }

    std::unique_ptr<intermediate> run(
            const std::atomic_bool& keep_going) const {
        //  RAYTRACER  /////////////////////////////////////////////////////////

        const auto rays_to_visualise = std::min(size_t{32}, raytracer_.rays);

        fprintf(stderr, "[engine] run: rays=%zu img_src_order=%zu room_vol=%.1f\n",
                raytracer_.rays, raytracer_.maximum_image_source_order,
                room_volume_);
        fflush(stderr);

        // Cap ray count to prevent GPU memory exhaustion on 6GB cards.
        // Each ray needs ~56 bytes of buffer; 100K rays ≈ 5.6 MB which is safe.
        constexpr size_t MAX_RAYS = 100000;
        if (raytracer_.rays > MAX_RAYS) {
            fprintf(stderr, "[engine] Capping rays %zu -> %zu (VRAM limit)\n",
                    raytracer_.rays, MAX_RAYS);
            fflush(stderr);
            raytracer_.rays = MAX_RAYS;
        }

        engine_state_changed_(state::starting_raytracer, 1.0);

        fprintf(stderr, "[engine] run: calling raytracer::canonical\n"); fflush(stderr);

        auto raytracer_output = raytracer::canonical(
                compute_context_,
                voxels_and_mesh_.voxels,
                source_,
                receiver_,
                environment_,
                raytracer_,
                rays_to_visualise,
                keep_going,
                [&](auto step, auto total_steps) {
                    if (step % 10 == 0) {
                        fprintf(stderr, "[engine] raytracer step %zu / %zu\n",
                                (size_t)step, (size_t)total_steps);
                        fflush(stderr);
                    }
                    engine_state_changed_(state::running_raytracer,
                                          step / (total_steps - 1.0));
                });

        fprintf(stderr, "[engine] run: raytracer::canonical done, output=%s\n",
                raytracer_output ? "OK" : "NULL"); fflush(stderr);

        if (!(keep_going && raytracer_output)) {
            return nullptr;
        }

        engine_state_changed_(state::finishing_raytracer, 1.0);

        //  Apply source directivity weighting to image source impulses.
        //  Each impulse has a `position` field (the image source location).
        //  The emission direction is source → image_source_position.
        //  After multiple bounces, energy becomes diffuse, so directivity
        //  primarily affects early reflections (image source path).
        if (source_directivity_ !=
            model::directivity_pattern::omnidirectional) {
            const auto fwd = source_orientation_.get_pointing();
            size_t weighted = 0;
            for (auto& imp : raytracer_output->aural.image_source) {
                const glm::vec3 imp_pos{imp.position.s[0],
                                        imp.position.s[1],
                                        imp.position.s[2]};
                const auto dir = imp_pos - source_;
                const auto len = glm::length(dir);
                if (len < 1e-6f) continue;
                const auto cos_theta = glm::dot(dir / len, fwd);
                const auto gain = model::directivity_gain(
                        source_directivity_, cos_theta);
                for (int b = 0; b < core::simulation_bands; ++b) {
                    imp.volume.s[b] *= gain;
                }
                ++weighted;
            }
            fprintf(stderr,
                    "[engine] directivity weighting: pattern=%d, "
                    "weighted %zu/%zu image source impulses\n",
                    static_cast<int>(source_directivity_),
                    weighted,
                    raytracer_output->aural.image_source.size());
            fflush(stderr);
        }

        raytracer_reflections_generated_(std::move(raytracer_output->visual),
                                         source_);

        //  look for the max time of an impulse
        const auto max_stochastic_time =
                max_time(raytracer_output->aural.stochastic);

        //  Waveguide duration: use whichever is longer — the raytracer's
        //  geometric simulation time or the Sabine RT60 estimate from
        //  the actual room geometry and materials.  This ensures the
        //  waveguide captures the full low-frequency decay, which can
        //  exceed the geometric prediction due to modal behavior.
        //  Cap at min(sabine_rt60 * 1.5, 15s) — allows longer tails for
        //  reverberant rooms while preventing runaway on RTX 2060 6GB.
        const auto waveguide_cap = std::min(sabine_rt60_max_ * 1.5, 15.0);
        const auto waveguide_time = std::min(
                std::max(max_stochastic_time, sabine_rt60_max_), waveguide_cap);

        fprintf(stderr, "[engine] run: max_stochastic=%.4f s  sabine_rt60=%.4f s  waveguide_time=%.4f s\n",
                max_stochastic_time, sabine_rt60_max_, waveguide_time); fflush(stderr);

        //  WAVEGUIDE  /////////////////////////////////////////////////////////
        engine_state_changed_(state::starting_waveguide, 1.0);

        fprintf(stderr, "[engine] run: calling waveguide_->run\n"); fflush(stderr);

        auto waveguide_output = waveguide_->run(
                compute_context_,
                voxels_and_mesh_,
                source_,
                receiver_,
                environment_,
                waveguide_time,
                keep_going,
                [&](auto& queue, const auto& buffer, auto step, auto steps) {
                    if (step % 100 == 0) {
                        fprintf(stderr, "[engine] waveguide step %zu / %zu\n",
                                (size_t)step, (size_t)steps);
                        fflush(stderr);
                    }
                    //  If there are node pressure listeners.
                    if (!waveguide_node_pressures_changed_.empty()) {
                        auto pressures =
                                core::read_from_buffer<float>(queue, buffer);
                        const auto time =
                                step / waveguide_->compute_sampling_frequency();
                        const auto distance =
                                time * environment_.speed_of_sound;
                        waveguide_node_pressures_changed_(std::move(pressures),
                                                          distance);
                    }

                    engine_state_changed_(state::running_waveguide,
                                          step / (steps - 1.0));
                });

        fprintf(stderr, "[engine] run: waveguide done, output=%s\n",
                waveguide_output ? "OK" : "NULL"); fflush(stderr);

        if (!(keep_going && waveguide_output)) {
            return nullptr;
        }

        engine_state_changed_(state::finishing_waveguide, 1.0);

        return make_intermediate_impl_ptr(
                make_combined_results(std::move(raytracer_output->aural),
                                      std::move(*waveguide_output)),
                source_,
                receiver_,
                room_volume_,
                environment_);
    }

    //  notifications  /////////////////////////////////////////////////////////

    engine_state_changed::connection connect_engine_state_changed(
            engine_state_changed::callback_type callback) {
        return engine_state_changed_.connect(std::move(callback));
    }

    waveguide_node_pressures_changed::connection
    connect_waveguide_node_pressures_changed(
            waveguide_node_pressures_changed::callback_type callback) {
        return waveguide_node_pressures_changed_.connect(std::move(callback));
    }

    raytracer_reflections_generated::connection
    connect_raytracer_reflections_generated(
            raytracer_reflections_generated::callback_type callback) {
        return raytracer_reflections_generated_.connect(std::move(callback));
    }

    //  cached data  ///////////////////////////////////////////////////////////

    const waveguide::voxels_and_mesh& get_voxels_and_mesh() const {
        return voxels_and_mesh_;
    }

private:
    core::compute_context compute_context_;
    waveguide::voxels_and_mesh voxels_and_mesh_;
    double room_volume_;
    double sabine_rt60_max_ = 2.0;
    glm::vec3 source_;
    glm::vec3 receiver_;
    core::environment environment_;
    mutable raytracer::simulation_parameters raytracer_;
    std::unique_ptr<waveguide_base> waveguide_;
    model::directivity_pattern source_directivity_;
    core::orientation source_orientation_;

    engine_state_changed engine_state_changed_;
    waveguide_node_pressures_changed waveguide_node_pressures_changed_;
    raytracer_reflections_generated raytracer_reflections_generated_;
};

////////////////////////////////////////////////////////////////////////////////

engine::engine(const core::compute_context& compute_context,
               const core::gpu_scene_data& scene_data,
               const glm::vec3& source,
               const glm::vec3& receiver,
               const core::environment& environment,
               const raytracer::simulation_parameters& raytracer,
               std::unique_ptr<waveguide_base> waveguide,
               model::directivity_pattern source_directivity,
               const core::orientation& source_orientation)
        : pimpl_{std::make_unique<impl>(compute_context,
                                        scene_data,
                                        source,
                                        receiver,
                                        environment,
                                        raytracer,
                                        std::move(waveguide),
                                        source_directivity,
                                        source_orientation)} {}

engine::~engine() noexcept = default;

std::unique_ptr<intermediate> engine::run(
        const std::atomic_bool& keep_going) const {
    return pimpl_->run(keep_going);
}

engine::engine_state_changed::connection engine::connect_engine_state_changed(
        engine_state_changed::callback_type callback) {
    return pimpl_->connect_engine_state_changed(std::move(callback));
}

engine::waveguide_node_pressures_changed::connection
engine::connect_waveguide_node_pressures_changed(
        waveguide_node_pressures_changed::callback_type callback) {
    return pimpl_->connect_waveguide_node_pressures_changed(
            std::move(callback));
}

engine::raytracer_reflections_generated::connection
engine::connect_raytracer_reflections_generated(
        raytracer_reflections_generated::callback_type callback) {
    return pimpl_->connect_raytracer_reflections_generated(std::move(callback));
}

const waveguide::voxels_and_mesh& engine::get_voxels_and_mesh() const {
    return pimpl_->get_voxels_and_mesh();
}

}  // namespace combined
}  // namespace wayverb
