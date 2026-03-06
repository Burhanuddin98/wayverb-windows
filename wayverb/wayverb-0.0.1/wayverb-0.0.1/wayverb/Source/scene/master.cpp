#include "master.h"

#include "main_model.h"

#include "controller.h"
#include "view.h"

#include "core/conversions.h"
#include "core/geo/box.h"
#include "core/reverb_time.h"

#include "raytracer/cl/reflection.h"

#include "AngularLookAndFeel.h"
#include "Application.h"
#include "CommandIDs.h"

#include "../UtilityComponents/generic_renderer.h"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_inverse.hpp"
#include "glm/gtc/matrix_transform.hpp"
#include "glm/gtc/quaternion.hpp"
#include "glm/gtc/type_ptr.hpp"

#include <mutex>

namespace scene {

namespace {

/// Cached FDTD pressure snapshot for replay.
struct fdtd_frame {
    util::aligned::vector<float> pressures;
    float distance;
};

/// Two-row bottom bar:
///   Row 1 -- Dimensions (left) + Ray controls (right)
///   Row 2 -- FDTD replay controls (label + Play/Pause + Replay + Speed)
class scene_bottom_bar final : public Component,
                               public Timer,
                               public Button::Listener,
                               public Slider::Listener {
public:
    scene_bottom_bar() {
        setOpaque(true);

        // --- Ray controls ---
        ray_play_btn_.setButtonText("Play");
        ray_replay_btn_.setButtonText("Replay");
        ray_hide_btn_.setButtonText("Hide Rays");
        ray_speed_slider_.setRange(0.2, 5.0, 0.1);
        ray_speed_slider_.setValue(1.0, dontSendNotification);
        ray_speed_slider_.setSliderStyle(Slider::LinearHorizontal);
        ray_speed_slider_.setTextBoxStyle(Slider::TextBoxRight, false, 36, 18);
        ray_speed_slider_.addListener(this);
        ray_speed_label_.setText("Speed:", dontSendNotification);
        ray_speed_label_.setFont(Font(10.0f));

        ray_play_btn_.addListener(this);
        ray_replay_btn_.addListener(this);
        ray_hide_btn_.addListener(this);

        // --- FDTD controls ---
        fdtd_play_btn_.setButtonText("Play");
        fdtd_replay_btn_.setButtonText("Replay");
        fdtd_hide_btn_.setButtonText("Hide FDTD");
        fdtd_speed_slider_.setRange(0.2, 5.0, 0.1);
        fdtd_speed_slider_.setValue(1.0, dontSendNotification);
        fdtd_speed_slider_.setSliderStyle(Slider::LinearHorizontal);
        fdtd_speed_slider_.setTextBoxStyle(Slider::TextBoxRight, false, 36, 18);
        fdtd_speed_slider_.addListener(this);
        fdtd_speed_label_.setText("Speed:", dontSendNotification);
        fdtd_speed_label_.setFont(Font(10.0f));

        fdtd_play_btn_.addListener(this);
        fdtd_replay_btn_.addListener(this);
        fdtd_hide_btn_.addListener(this);

        // --- Labels ---
        dim_label_.setFont(Font(11.0f));
        dim_label_.setJustificationType(Justification::centredLeft);

        ray_label_.setText("Rays", dontSendNotification);
        ray_label_.setFont(Font(10.0f, Font::bold));
        ray_label_.setJustificationType(Justification::centredLeft);

        fdtd_label_.setText("FDTD", dontSendNotification);
        fdtd_label_.setFont(Font(10.0f, Font::bold));
        fdtd_label_.setJustificationType(Justification::centredLeft);

        addAndMakeVisible(dim_label_);
        addAndMakeVisible(ray_label_);
        addAndMakeVisible(ray_play_btn_);
        addAndMakeVisible(ray_replay_btn_);
        addAndMakeVisible(ray_hide_btn_);
        addAndMakeVisible(ray_speed_label_);
        addAndMakeVisible(ray_speed_slider_);
        addAndMakeVisible(fdtd_label_);
        addAndMakeVisible(fdtd_play_btn_);
        addAndMakeVisible(fdtd_replay_btn_);
        addAndMakeVisible(fdtd_hide_btn_);
        addAndMakeVisible(fdtd_speed_label_);
        addAndMakeVisible(fdtd_speed_slider_);
    }

    void set_dimensions(glm::vec3 dim) {
        room_dim_ = dim;
        update_dim_text();
    }

    // --- Ray animation ---
    void start_ray_animation(double max_dist,
                             std::function<void(float)> set_distance) {
        ray_set_distance_ = std::move(set_distance);
        has_reflections_ = true;
        max_ray_distance_ = max_dist;
        current_ray_distance_ = 0;
        ray_playing_ = true;
        rays_visible_ = true;
        ray_play_btn_.setButtonText("Pause");
        ray_hide_btn_.setButtonText("Hide Rays");
        ensure_timer();
        repaint();
    }

    void reset_ray_animation() {
        has_reflections_ = false;
        ray_playing_ = false;
        current_ray_distance_ = 0;
        max_ray_distance_ = 0;
        ray_play_btn_.setButtonText("Play");
        check_stop_timer();
        repaint();
    }

    // --- FDTD frame cache ---
    void set_fdtd_send_frame(
            std::function<void(const util::aligned::vector<float>&)> fn) {
        fdtd_send_frame_ = std::move(fn);
    }

    void add_fdtd_frame(util::aligned::vector<float> pressures, float distance) {
        // Store exactly the first 600 frames, ignore the rest.
        if (fdtd_frames_.size() >= max_fdtd_frames_) return;
        fdtd_frames_.push_back({std::move(pressures), distance});
    }

    void clear_fdtd_frames() {
        fdtd_frames_.clear();
        fdtd_fractional_ = 0;
        fdtd_playing_ = false;
        fdtd_visible_ = true;
        fdtd_play_btn_.setButtonText("Play");
        fdtd_hide_btn_.setButtonText("Hide FDTD");
        check_stop_timer();
    }

    void finish_fdtd_recording() {
        // Auto-start replay
        if (!fdtd_frames_.empty()) {
            fdtd_fractional_ = 0;
            fdtd_playing_ = true;
            fdtd_play_btn_.setButtonText("Pause");
            ensure_timer();
        }
    }

    // --- Layout ---
    void resized() override {
        const int rowH = 22, pad = 4, btnW = 60;
        auto area = getLocalBounds().reduced(4, 2);

        auto row1 = area.removeFromTop(rowH);
        auto row2 = area;

        dim_label_.setBounds(row1.removeFromLeft(jmin(row1.getWidth() / 3, 240)));
        row1.removeFromLeft(pad);
        ray_label_.setBounds(row1.removeFromLeft(30));
        row1.removeFromLeft(pad);
        ray_play_btn_.setBounds(row1.removeFromLeft(btnW));
        row1.removeFromLeft(pad);
        ray_replay_btn_.setBounds(row1.removeFromLeft(btnW));
        row1.removeFromLeft(pad);
        ray_hide_btn_.setBounds(row1.removeFromLeft(btnW + 5));
        row1.removeFromLeft(pad);
        ray_speed_label_.setBounds(row1.removeFromLeft(38));
        ray_speed_slider_.setBounds(row1.removeFromLeft(jmin(row1.getWidth(), 90)));

        auto fdtd_area = row2;
        fdtd_area.removeFromLeft(jmin(row2.getWidth() / 3, 240) + pad);
        fdtd_label_.setBounds(fdtd_area.removeFromLeft(30));
        fdtd_area.removeFromLeft(pad);
        fdtd_play_btn_.setBounds(fdtd_area.removeFromLeft(btnW));
        fdtd_area.removeFromLeft(pad);
        fdtd_replay_btn_.setBounds(fdtd_area.removeFromLeft(btnW));
        fdtd_area.removeFromLeft(pad);
        fdtd_hide_btn_.setBounds(fdtd_area.removeFromLeft(btnW + 5));
        fdtd_area.removeFromLeft(pad);
        fdtd_speed_label_.setBounds(fdtd_area.removeFromLeft(38));
        fdtd_speed_slider_.setBounds(fdtd_area.removeFromLeft(jmin(fdtd_area.getWidth(), 90)));
    }

    void paint(Graphics& g) override {
        g.fillAll(Colours::darkgrey.darker(0.8f));

        g.setColour(Colours::white.withAlpha(0.15f));
        g.drawHorizontalLine(0, 0.0f, float(getWidth()));

        int midY = getHeight() / 2;
        g.setColour(Colours::white.withAlpha(0.08f));
        g.drawHorizontalLine(midY, 4.0f, float(getWidth() - 4));

        if (has_reflections_ && max_ray_distance_ > 0) {
            float progress = float(current_ray_distance_ / max_ray_distance_);
            g.setColour(AngularLookAndFeel::emphasis);
            g.fillRect(0, midY - 2, int(getWidth() * progress), 2);
        }

        if (!fdtd_frames_.empty()) {
            float progress = float(fdtd_fractional_ / double(fdtd_frames_.size() - 1));
            g.setColour(Colour(0xffff6600));
            g.fillRect(0, getHeight() - 2, int(getWidth() * progress), 2);
        }
    }

    void timerCallback() override {
        if (ray_playing_ && max_ray_distance_ > 0) {
            double speed = ray_speed_slider_.getValue();
            double step = (max_ray_distance_ / 3.0) * speed / 30.0;
            current_ray_distance_ += step;

            if (current_ray_distance_ >= max_ray_distance_) {
                current_ray_distance_ = max_ray_distance_;
                ray_playing_ = false;
                ray_play_btn_.setButtonText("Play");
            }

            if (ray_set_distance_ && rays_visible_)
                ray_set_distance_(float(current_ray_distance_));
        }

        if (fdtd_playing_ && fdtd_frames_.size() >= 2) {
            double speed = fdtd_speed_slider_.getValue();
            fdtd_fractional_ += speed;

            double max_idx = double(fdtd_frames_.size() - 1);
            if (fdtd_fractional_ >= max_idx) {
                fdtd_fractional_ = max_idx;
                fdtd_playing_ = false;
                fdtd_play_btn_.setButtonText("Play");
            }

            if (fdtd_visible_)
                send_interpolated_frame(fdtd_fractional_);
        }

        if (!ray_playing_ && !fdtd_playing_)
            check_stop_timer();

        repaint();
    }

    void buttonClicked(Button* b) override {
        if (b == &ray_play_btn_) {
            if (!has_reflections_) return;
            if (ray_playing_) {
                ray_playing_ = false;
                ray_play_btn_.setButtonText("Play");
            } else {
                if (current_ray_distance_ >= max_ray_distance_)
                    current_ray_distance_ = 0;
                ray_playing_ = true;
                ray_play_btn_.setButtonText("Pause");
                ensure_timer();
            }
        } else if (b == &ray_replay_btn_) {
            if (!has_reflections_) return;
            current_ray_distance_ = 0;
            ray_playing_ = true;
            ray_play_btn_.setButtonText("Pause");
            ensure_timer();
        } else if (b == &ray_hide_btn_) {
            if (!has_reflections_) return;
            rays_visible_ = !rays_visible_;
            ray_hide_btn_.setButtonText(rays_visible_ ? "Hide Rays" : "Show Rays");
            if (ray_set_distance_)
                ray_set_distance_(rays_visible_ ? float(current_ray_distance_) : 0);
        }
        else if (b == &fdtd_play_btn_) {
            if (fdtd_frames_.size() < 2) return;
            if (fdtd_playing_) {
                fdtd_playing_ = false;
                fdtd_play_btn_.setButtonText("Play");
            } else {
                if (fdtd_fractional_ >= double(fdtd_frames_.size() - 1))
                    fdtd_fractional_ = 0;
                fdtd_playing_ = true;
                fdtd_play_btn_.setButtonText("Pause");
                ensure_timer();
            }
        } else if (b == &fdtd_replay_btn_) {
            if (fdtd_frames_.size() < 2) return;
            fdtd_fractional_ = 0;
            fdtd_playing_ = true;
            fdtd_play_btn_.setButtonText("Pause");
            ensure_timer();
        } else if (b == &fdtd_hide_btn_) {
            if (fdtd_frames_.empty()) return;
            fdtd_visible_ = !fdtd_visible_;
            fdtd_hide_btn_.setButtonText(fdtd_visible_ ? "Hide FDTD" : "Show FDTD");
            if (!fdtd_visible_ && fdtd_send_frame_) {
                // Send zero pressures to hide
                util::aligned::vector<float> zeros(
                        fdtd_frames_[0].pressures.size(), 0.0f);
                fdtd_send_frame_(zeros);
            } else if (fdtd_visible_) {
                send_interpolated_frame(fdtd_fractional_);
            }
        }
    }

    void sliderValueChanged(Slider*) override {}

private:
    void ensure_timer() {
        if (!isTimerRunning()) startTimerHz(30);
    }
    void check_stop_timer() {
        if (!ray_playing_ && !fdtd_playing_) stopTimer();
    }

    /// Interpolate between two cached frames and send to renderer.
    void send_interpolated_frame(double fractional_idx) {
        if (fdtd_frames_.empty() || !fdtd_send_frame_) return;

        int idx_a = int(fractional_idx);
        int idx_b = idx_a + 1;
        float t = float(fractional_idx - idx_a);

        if (idx_a < 0) idx_a = 0;
        if (idx_b >= int(fdtd_frames_.size()))
            idx_b = int(fdtd_frames_.size()) - 1;
        if (idx_a >= int(fdtd_frames_.size()))
            idx_a = int(fdtd_frames_.size()) - 1;

        const auto& fa = fdtd_frames_[idx_a].pressures;
        const auto& fb = fdtd_frames_[idx_b].pressures;

        if (fa.size() != fb.size() || fa.empty()) {
            fdtd_send_frame_(fa);
            return;
        }

        // Linear interpolation between frames
        util::aligned::vector<float> interp(fa.size());
        for (size_t i = 0; i < fa.size(); ++i) {
            interp[i] = fa[i] * (1.0f - t) + fb[i] * t;
        }
        fdtd_send_frame_(interp);
    }

    void update_dim_text() {
        auto fmt = [](const char* axis, float v) -> String {
            if (v >= 1.0f)
                return String(axis) + String(v, 2) + " m";
            return String(axis) + String(v * 100.0f, 1) + " cm";
        };
        dim_label_.setText(fmt("X: ", room_dim_.x) + "    " +
                                   fmt("Y: ", room_dim_.y) + "    " +
                                   fmt("Z: ", room_dim_.z),
                           dontSendNotification);
    }

    glm::vec3 room_dim_{0, 0, 0};

    bool has_reflections_ = false;
    bool ray_playing_ = false;
    bool rays_visible_ = true;
    double max_ray_distance_ = 0;
    double current_ray_distance_ = 0;
    std::function<void(float)> ray_set_distance_;

    std::vector<fdtd_frame> fdtd_frames_;
    double fdtd_fractional_ = 0;
    bool fdtd_playing_ = false;
    bool fdtd_visible_ = true;
    static constexpr size_t max_fdtd_frames_ = 600;
    std::function<void(const util::aligned::vector<float>&)> fdtd_send_frame_;

    Label dim_label_;
    Label ray_label_;
    TextButton ray_play_btn_;
    TextButton ray_replay_btn_;
    TextButton ray_hide_btn_;
    Label ray_speed_label_;
    Slider ray_speed_slider_;

    Label fdtd_label_;
    TextButton fdtd_play_btn_;
    TextButton fdtd_replay_btn_;
    TextButton fdtd_hide_btn_;
    Label fdtd_speed_label_;
    Slider fdtd_speed_slider_;
};

}  // namespace

class master::impl final : public Component,
                           public generic_renderer<view>::Listener,
                           public SettableTooltipClient {
public:
    impl(main_model& model)
            : model_{model}
            , controller_{model_}
            , visible_surface_changed_connection_{
                model_.scene.connect_visible_surface_changed([this](auto visible) {
                    view_.high_priority_command([=](auto& r) { r.set_highlighted_surface(visible); });
                })}
            , view_state_changed_connection_{
                model_.scene.connect_view_state_changed([this](auto state) {
                    view_.high_priority_command([=](auto& r) { r.set_view_state(state); });
                })}
            , projection_matrix_changed_connection_{
                model_.scene.connect_projection_matrix_changed([this](auto matrix) {
                    view_.high_priority_command([=](auto& r) { r.set_projection_matrix(matrix); });
                })}
            , visualise_changed_connection_{
                 model_.scene.connect_visualise_changed([this](auto should_visualise) {
                    if (should_visualise) {
                        positions_changed_ = main_model::waveguide_node_positions_changed::scoped_connection{
                                        model_.connect_node_positions([this](
                                                auto descriptor) {
                                            view_.high_priority_command([d = std::move(descriptor)](
                                                    auto& renderer) {
                                                renderer.set_node_positions(
                                                        wayverb::waveguide::
                                                                compute_node_positions(
                                                                        d));
                                            });
                                        })};

                        pressures_changed_ = main_model::waveguide_node_pressures_changed::scoped_connection{
                                        model_.connect_node_pressures([this](
                                                auto pressures, auto distance) {
                                            // Cache for replay
                                            bottom_bar_.add_fdtd_frame(pressures, distance);

                                            // Live display
                                            view_.low_priority_command([
                                                p = std::move(pressures),
                                                d = distance
                                            ](auto& renderer) {
                                                renderer.set_node_pressures(std::move(p));
                                                renderer.set_distance_travelled(d);
                                            });
                                        })};

                        reflections_generated_ =
                                main_model::raytracer_reflections_generated::scoped_connection{model_.connect_reflections(
                                                [this](auto reflections, auto source) {
                                                    float max_d = 0;
                                                    for (const auto& ray : reflections) {
                                                        float d = 0;
                                                        glm::vec3 prev = source;
                                                        for (const auto& r : ray) {
                                                            auto pos = wayverb::core::to_vec3{}(r.position);
                                                            d += glm::distance(prev, pos);
                                                            prev = pos;
                                                        }
                                                        max_d = std::max(max_d, d);
                                                    }

                                                    view_.high_priority_command([
                                                        r = std::move(reflections),
                                                        s = source
                                                    ](auto& renderer) {
                                                        renderer.set_reflections(
                                                                std::move(r), s);
                                                    });

                                                    auto max_dist = double(max_d);
                                                    MessageManager::callAsync([this, max_dist] {
                                                        bottom_bar_.start_ray_animation(max_dist,
                                                            [this](float d) {
                                                                view_.low_priority_command([d](auto& renderer) {
                                                                    renderer.set_distance_travelled(d);
                                                                });
                                                            });
                                                    });
                                                })};

                    } else {
                        positions_changed_     = main_model::waveguide_node_positions_changed::scoped_connection{};
                        pressures_changed_     = main_model::waveguide_node_pressures_changed::scoped_connection{};
                        reflections_generated_ = main_model::raytracer_reflections_generated::scoped_connection{};
                        view_.high_priority_command([](auto& renderer) { renderer.clear(); });
                        bottom_bar_.reset_ray_animation();
                        bottom_bar_.clear_fdtd_frames();
                    }
                })}
            , sources_connection_{
                model_.project.persistent.sources()->connect([this](auto& sources) {
                    auto copy = sources;
                    view_.high_priority_command([r = std::move(copy)](auto& renderer) {
                        renderer.set_sources(std::move(r));
                    });
                })}
            , receivers_connection_{
                model_.project.persistent.receivers()->connect([this](auto& receivers) {
                    auto copy = receivers;
                    view_.high_priority_command([r = std::move(copy)](auto& renderer) {
                        renderer.set_receivers(std::move(r));
                    });
                })}
            , begun_{model_.connect_begun([this] {
                    setEnabled(false);
                    bottom_bar_.clear_fdtd_frames();
                })}
            , finished_{model_.connect_finished([this] {
                    setEnabled(true);
                    bottom_bar_.finish_fdtd_recording();
                })}
            {
        model_.project.persistent.sources()->notify();
        model_.project.persistent.receivers()->notify();

        // Wire FDTD replay: push interpolated pressures to the renderer
        bottom_bar_.set_fdtd_send_frame(
            [this](const util::aligned::vector<float>& pressures) {
                auto p_copy = pressures;
                view_.low_priority_command([
                    p = std::move(p_copy)
                ](auto& renderer) {
                    renderer.set_node_pressures(std::move(p));
                });
            });

        auto rebuild_scene = [this] {
            const auto scene_data = model_.project.get_scene_data();
            auto triangles = scene_data.get_triangles();
            auto vertices = util::map_to_vector(
                    begin(scene_data.get_vertices()),
                    end(scene_data.get_vertices()),
                    wayverb::core::to_vec3{});
            view_.high_priority_command([
                t = std::move(triangles),
                v = std::move(vertices)
            ](auto& r) {
                r.set_scene(t.data(), t.size(), v.data(), v.size());
            });
        };
        controller_.on_scene_changed = rebuild_scene;
        model_.project.on_scale_changed = [rebuild_scene] {
            MessageManager::callAsync(rebuild_scene);
        };

        view_.setInterceptsMouseClicks(false, false);

        addAndMakeVisible(view_);
        addAndMakeVisible(bottom_bar_);

        update_dimensions();

        setTooltip("left-click: move sources and receivers\n"
               "right-click: rotate view\n"
               "middle-click: pan view\n"
               "scroll: zoom view");
    }

    void resized() override {
        auto area = getLocalBounds();
        bottom_bar_.setBounds(area.removeFromBottom(50));
        view_.setBounds(area);
        model_.scene.set_viewport(glm::vec2{area.getWidth(), area.getHeight()});
        update_dimensions();
    }

    void enablementChanged() override {
        controller_.enablement_changed(isEnabled());
    }

    void mouseMove(const MouseEvent& e) override { controller_.mouse_move(e); }
    void mouseDown(const MouseEvent& e) override { controller_.mouse_down(e); }
    void mouseDrag(const MouseEvent& e) override { controller_.mouse_drag(e); }
    void mouseUp(const MouseEvent& e) override { controller_.mouse_up(e); }

    void mouseWheelMove(const MouseEvent& e,
                        const MouseWheelDetails& d) override {
        controller_.mouse_wheel_move(e, d);
    }

    void context_created(generic_renderer<scene::view>&) override {
        const auto scene_data = model_.project.get_scene_data();
        auto triangles = scene_data.get_triangles();
        auto vertices = util::map_to_vector(begin(scene_data.get_vertices()),
                                            end(scene_data.get_vertices()),
                                            wayverb::core::to_vec3{});

        view_.high_priority_command([
            t = std::move(triangles),
            v = std::move(vertices),
            vs = model_.scene.get_view_state(),
            pm = model_.scene.get_projection_matrix()
        ](auto& r) {
            r.set_scene(t.data(), t.size(), v.data(), v.size());
            r.set_view_state(vs);
            r.set_projection_matrix(pm);
            r.set_emphasis_colour(
                    {AngularLookAndFeel::emphasis.getFloatRed(),
                     AngularLookAndFeel::emphasis.getFloatGreen(),
                     AngularLookAndFeel::emphasis.getFloatBlue()});
        });
    }

    void context_closing(generic_renderer<scene::view>&) override {}

private:
    void update_dimensions() {
        const auto scene_data = model_.project.get_scene_data();
        const auto aabb = wayverb::core::geo::compute_aabb(
                scene_data.get_vertices());
        bottom_bar_.set_dimensions(dimensions(aabb));
    }

    main_model& model_;

    generic_renderer<scene::view> view_;
    model::Connector<generic_renderer<scene::view>> view_connector_{&view_, this};

    scene::controller controller_;

    scene_bottom_bar bottom_bar_;

    wayverb::combined::model::scene::visible_surface_changed::scoped_connection
            visible_surface_changed_connection_;
    wayverb::combined::model::scene::view_state_changed::scoped_connection
            view_state_changed_connection_;
    wayverb::combined::model::scene::projection_matrix_changed::
            scoped_connection projection_matrix_changed_connection_;
    wayverb::combined::model::scene::visualise_changed::scoped_connection
            visualise_changed_connection_;
    wayverb::combined::model::sources::scoped_connection sources_connection_;
    wayverb::combined::model::receivers::scoped_connection
            receivers_connection_;

    main_model::waveguide_node_positions_changed::scoped_connection positions_changed_;
    main_model::waveguide_node_pressures_changed::scoped_connection pressures_changed_;
    main_model::raytracer_reflections_generated::scoped_connection reflections_generated_;
    main_model::begun::scoped_connection begun_;
    main_model::finished::scoped_connection finished_;
};

master::master(main_model& model)
        : pimpl_{std::make_unique<impl>(model)} {
    addAndMakeVisible(*pimpl_);
}

master::~master() noexcept = default;

void master::resized() { pimpl_->setBounds(getLocalBounds()); }

}  // namespace scene
