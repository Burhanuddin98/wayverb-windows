#include "controller.h"

#include "utilities/map_to_vector.h"

namespace scene {

/// A mouse action is a series of movements while a mouse button is held.
class controller::mouse_action {
public:
    /// Construct on mouse-down.
    mouse_action() = default;

    mouse_action(const mouse_action&) = default;
    mouse_action(mouse_action&&) noexcept = default;

    mouse_action& operator=(const mouse_action&) = default;
    mouse_action& operator=(mouse_action&&) noexcept = default;

    /// Destroy on mouse-up.
    virtual ~mouse_action() noexcept = default;

    /// Do something when the mouse is moved.
    virtual void mouse_drag(const MouseEvent& e) = 0;
};

////////////////////////////////////////////////////////////////////////////////

class rotate_action final : public controller::mouse_action {
public:
    rotate_action(wayverb::combined::model::scene& model)
            : model_{model}
            , initial_{model_.get_rotation()} {}

    void mouse_drag(const MouseEvent& e) override {
        const auto diff = e.getOffsetFromDragStart();
        const auto angle_scale = 0.005;
        model_.set_rotation(wayverb::core::az_el{
                static_cast<float>(diff.x * angle_scale + initial_.azimuth),
                static_cast<float>(diff.y * angle_scale + initial_.elevation)});
    }

private:
    wayverb::combined::model::scene& model_;
    wayverb::core::az_el initial_;
};

////////////////////////////////////////////////////////////////////////////////

class pan_action final : public controller::mouse_action {
public:
    pan_action(wayverb::combined::model::scene& model)
            : model_{model}
            , camera_position_{model_.compute_world_camera_position()}
            , camera_direction_{model_.compute_world_camera_direction()}
            , initial_position_{model_.get_origin()}
            , camera_distance_{
                      glm::distance(initial_position_, camera_position_)} {}

    void mouse_drag(const MouseEvent& e) override {
        //  Find mouse position on plane perpendicular to camera direction
        //  through the rotation origin.
        const auto initial = compute_world_mouse_position(glm::vec2{
                e.getMouseDownPosition().x, e.getMouseDownPosition().y});
        const auto current = compute_world_mouse_position(
                glm::vec2{e.getPosition().x, e.getPosition().y});
        //  Adjust the model origin based on the difference between the initial
        //  and current mouse positions.
        model_.set_origin(initial_position_ + current - initial);
    }

private:
    glm::vec3 compute_world_mouse_position(const glm::vec2& mouse_pos) {
        const auto mouse_direction =
                model_.compute_world_mouse_direction(mouse_pos);
        return camera_position_ +
               mouse_direction * camera_distance_ /
                       glm::dot(camera_direction_, mouse_direction);
    }

    wayverb::combined::model::scene& model_;
    glm::vec3 camera_position_;
    glm::vec3 camera_direction_;
    glm::vec3 initial_position_;
    float camera_distance_;
};

////////////////////////////////////////////////////////////////////////////////

namespace {

std::ostream& operator<<(std::ostream& os, const glm::vec3& p) {
    return os << p.x << ", " << p.y << ", " << p.z;
}

}  // namespace

template <typename Item>
class move_item_action final : public controller::mouse_action {
public:
    move_item_action(const wayverb::combined::model::scene& model,
                     const std::shared_ptr<Item>& item)
            : model_{model}
            , item_{item}
            , camera_position_{model.compute_world_camera_position()}
            , camera_direction_{model.compute_world_camera_direction()}
            , initial_position_{item->get_position()}
            , camera_distance_{
                      glm::distance(initial_position_, camera_position_)} {
        item->hover_state()->set_selected(true);
    }

    void mouse_drag(const MouseEvent& e) override {
        if (auto strong = item_.lock()) {
            const auto initial = compute_world_mouse_position(glm::vec2{
                    e.getMouseDownPosition().x, e.getMouseDownPosition().y});
            const auto current = compute_world_mouse_position(
                    glm::vec2{e.getPosition().x, e.getPosition().y});
            const auto new_pos = initial_position_ + current - initial;
            strong->set_position(new_pos);
        }
    }

    ~move_item_action() noexcept {
        if (auto strong = item_.lock()) {
            strong->hover_state()->set_selected(false);
        }
    }

private:
    glm::vec3 compute_world_mouse_position(const glm::vec2& mouse_pos) {
        const auto mouse_direction =
                model_.compute_world_mouse_direction(mouse_pos);
        return camera_position_ +
               mouse_direction * camera_distance_ /
                       glm::dot(camera_direction_, mouse_direction);
    }

    const wayverb::combined::model::scene& model_;
    std::weak_ptr<Item> item_;
    glm::vec3 camera_position_;
    glm::vec3 camera_direction_;
    glm::vec3 initial_position_;
    float camera_distance_;
};

template <typename T>
auto make_move_item_action_ptr(wayverb::combined::model::scene& model,
                               const std::shared_ptr<T>& t) {
    return std::make_unique<move_item_action<T>>(model, t);
}

////////////////////////////////////////////////////////////////////////////////

controller::controller(main_model& model)
        : model_{model} {
    // Cache scene geometry for surface picking
    const auto scene_data = model_.project.get_scene_data();
    triangles_ = scene_data.get_triangles();
    vertices_ = util::map_to_vector(
            begin(scene_data.get_vertices()),
            end(scene_data.get_vertices()),
            wayverb::core::to_vec3{});
}

controller::~controller() noexcept = default;

void controller::enablement_changed(bool enabled) {
    allow_edit_ = enabled;
    if (!allow_edit_) {
        if (mouse_action_) {
            mouse_action_ = nullptr;
        }
    }
}

void controller::mouse_move(const MouseEvent& e) {
    if (allow_edit_) {
        //  Temporarily block connections to minimize redraw requests:

        model_.project.persistent.sources().block();
        model_.project.persistent.receivers().block();

        //  Ensure nothing is hovered.
        for (auto& i : *model_.project.persistent.sources()) {
            i->hover_state()->set_hovered(false);
        }
        for (auto& i : *model_.project.persistent.receivers()) {
            i->hover_state()->set_hovered(false);
        }

        //  Allow notifications to propagate again.
        model_.project.persistent.sources().unblock();
        model_.project.persistent.receivers().unblock();

        //  look for hovered items, notify if something is hovered.
        do_action_with_closest_thing(
                wayverb::core::to_vec2{}(e.getPosition()),
                *model_.project.persistent.sources(),
                *model_.project.persistent.receivers(),
                [](const auto& shared) {
                    shared->hover_state()->set_hovered(true);
                    return true;
                });
    }
}

void controller::mouse_down(const MouseEvent& e) {
    mouse_action_ = start_action(e);
}

void controller::mouse_drag(const MouseEvent& e) {
    //  Forward the position to the currently engaged action.
    if (mouse_action_) {
        mouse_action_->mouse_drag(e);
    }
}

void controller::mouse_up(const MouseEvent& e) {
    //  If a mouse button is released, quit the current action.
    //  If there are still buttons held, start the appropriate action for the
    //  remaining buttons.
    if (mouse_action_) {
        mouse_action_ = nullptr;
    }
}

void controller::mouse_wheel_move(const MouseEvent& e,
                                  const MouseWheelDetails& d) {
    //  Only zoom if another action is not ongoing.
    if (mouse_action_ == nullptr) {
        const auto current_distance = model_.scene.get_eye_distance();
        //  Smooth exponential zoom — 20% per scroll notch
        const auto factor = std::pow(0.8f, d.deltaY * 3.0f);
        const auto new_distance = std::max(0.01f, current_distance * factor);
        model_.scene.set_eye_distance(new_distance);
    }
}

std::unique_ptr<controller::mouse_action> controller::start_action(
        const MouseEvent& e) {
    //  Middle-click: pan
    if (e.mods.isMiddleButtonDown()) {
        return std::make_unique<pan_action>(model_.scene);
    }

    //  Right-click: pan (more intuitive secondary control)
    if (e.mods.isRightButtonDown()) {
        return std::make_unique<pan_action>(model_.scene);
    }

    //  Left-click: move hovered item, pick surface, or rotate
    if (e.mods.isLeftButtonDown()) {
        //  First: if a source/receiver is hovered, drag it
        if (allow_edit_) {
            auto action = do_action_with_closest_thing(
                    wayverb::core::to_vec2{}(e.getPosition()),
                    *model_.project.persistent.sources(),
                    *model_.project.persistent.receivers(),
                    [this](const auto& shared) -> std::unique_ptr<mouse_action> {
                        return make_move_item_action_ptr(model_.scene, shared);
                    });

            if (action) return action;

            //  Shift+left-click: surface picking for material assignment
            if (e.mods.isShiftDown()) {
                const auto mouse_pos = wayverb::core::to_vec2{}(e.getPosition());
                if (const auto surf = pick_surface(mouse_pos)) {
                    model_.scene.set_visible_surface(*surf);
                    show_material_popup(*surf);
                }
                return nullptr;
            }
        }

        //  Default left-drag on empty space: rotate the view
        return std::make_unique<rotate_action>(model_.scene);
    }

    return nullptr;
}

std::optional<size_t> controller::pick_surface(
        const glm::vec2& mouse_pos) const {
    if (triangles_.empty() || vertices_.empty()) {
        return std::nullopt;
    }

    const auto origin = model_.scene.compute_world_camera_position();
    const auto direction = model_.scene.compute_world_mouse_direction(mouse_pos);

    float best_t = std::numeric_limits<float>::max();
    std::optional<size_t> best_surface;

    for (const auto& tri : triangles_) {
        const auto& v0 = vertices_[tri.v0];
        const auto& v1 = vertices_[tri.v1];
        const auto& v2 = vertices_[tri.v2];

        // Möller-Trumbore ray-triangle intersection
        const auto e1 = v1 - v0;
        const auto e2 = v2 - v0;
        const auto pvec = glm::cross(direction, e2);
        const auto det = glm::dot(e1, pvec);

        if (std::abs(det) < 1e-8f) continue;

        const auto inv_det = 1.0f / det;
        const auto tvec = origin - v0;
        const auto u = glm::dot(tvec, pvec) * inv_det;
        if (u < 0.0f || u > 1.0f) continue;

        const auto qvec = glm::cross(tvec, e1);
        const auto v = glm::dot(direction, qvec) * inv_det;
        if (v < 0.0f || u + v > 1.0f) continue;

        const auto t = glm::dot(e2, qvec) * inv_det;
        if (t > 0.0f && t < best_t) {
            best_t = t;
            best_surface = tri.surface;
        }
    }

    return best_surface;
}

void controller::show_material_popup(size_t clicked_surface) {
    PopupMenu menu;
    const auto& materials = *model_.project.persistent.materials();

    // Header showing which region was clicked
    if (clicked_surface < materials.size()) {
        menu.addSectionHeader("Region: " +
                              materials[clicked_surface]->get_name());
        menu.addSeparator();
    }

    // Sub-header
    menu.addSectionHeader("Assign Material:");

    // Menu items: 1-based IDs (0 = dismissed)
    for (size_t i = 0; i < materials.size(); ++i) {
        const bool is_current = (i == clicked_surface);
        menu.addItem(static_cast<int>(i + 1),
                     materials[i]->get_name(),
                     !is_current,  // disabled if already assigned
                     is_current);  // ticked if current
    }

    menu.addSeparator();
    const int new_material_id = static_cast<int>(materials.size() + 1);
    menu.addItem(new_material_id, "New Material...");

    const int result = menu.show();
    if (result == 0) {
        // Dismissed — clear highlight
        model_.scene.set_visible_surface(std::nullopt);
        return;
    }

    size_t target_surface;
    if (result == new_material_id) {
        auto name = "Material " + std::to_string(materials.size() + 1);
        target_surface = model_.project.add_surface(name);
    } else {
        target_surface = static_cast<size_t>(result - 1);
    }

    model_.project.reassign_surface(clicked_surface, target_surface);
    model_.scene.set_visible_surface(target_surface);
    refresh_cached_geometry();

    if (on_scene_changed) {
        on_scene_changed();
    }
}

void controller::refresh_cached_geometry() {
    const auto scene_data = model_.project.get_scene_data();
    triangles_ = scene_data.get_triangles();
    vertices_ = util::map_to_vector(
            begin(scene_data.get_vertices()),
            end(scene_data.get_vertices()),
            wayverb::core::to_vec3{});
}

}  // namespace scene
