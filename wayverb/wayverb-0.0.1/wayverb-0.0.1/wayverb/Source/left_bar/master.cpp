#include "master.h"
#include "AngularLookAndFeel.h"

#include "materials/master.h"
#include "raytracer/master.h"
#include "receivers/master.h"
#include "sources/master.h"
#include "waveguide/master.h"

#include "core/reverb_time.h"

#include "utilities/string_builder.h"

#include <cmath>
#include <cstdio>

namespace {

//  Allows a generic component to be placed in a property panel.
//  Doesn't draw a label.
template <typename T>
class wrapped_property_component final : public PropertyComponent {
public:
    template <typename... Ts>
    wrapped_property_component(Ts&&... ts)
            : PropertyComponent{"empty"}
            , content{std::forward<Ts>(ts)...} {
        setLookAndFeel(&look_and_feel_);
        addAndMakeVisible(content);

        setPreferredHeight(content.getHeight());
    }

    void refresh() override {}

    void resized() override {
        content.setBounds(getLocalBounds());
    }

private:
    class look_and_feel final : public AngularLookAndFeel {
    public:
        //  Don't bother drawing anything.
        void drawPropertyComponentBackground(Graphics&,
                                             int,
                                             int,
                                             PropertyComponent&) override {}
        void drawPropertyComponentLabel(Graphics&,
                                        int,
                                        int,
                                        PropertyComponent&) override {}

        //  Let the content take up the entire space.
        Rectangle<int> getPropertyComponentContentPosition(
                PropertyComponent& c) override {
            return c.getLocalBounds();
        }
    };

    look_and_feel look_and_feel_;

public:
    T content;
};

////////////////////////////////////////////////////////////////////////////////

/// Button row embedded in the materials section of the PropertyPanel.
class material_buttons_component final : public Component,
                                          public TextButton::Listener {
public:
    material_buttons_component(std::function<void()> on_add,
                               std::function<void()> on_set_all)
            : on_add_{std::move(on_add)}
            , on_set_all_{std::move(on_set_all)} {
        add_btn_.addListener(this);
        set_all_btn_.addListener(this);
        addAndMakeVisible(add_btn_);
        addAndMakeVisible(set_all_btn_);

        add_btn_.setTooltip(
                "Add a new material. Click surfaces in the 3D view to assign.");
        set_all_btn_.setTooltip(
                "Apply the same material preset to every surface at once.");

        setSize(300, 28);
    }

    void resized() override {
        auto bounds = getLocalBounds().reduced(2, 2);
        auto left = bounds.removeFromLeft(bounds.getWidth() / 2);
        add_btn_.setBounds(left.reduced(1, 0));
        set_all_btn_.setBounds(bounds.reduced(1, 0));
    }

    void buttonClicked(Button* b) override {
        if (b == &add_btn_ && on_add_) on_add_();
        if (b == &set_all_btn_ && on_set_all_) on_set_all_();
    }

private:
    TextButton add_btn_{"+ Add Material"};
    TextButton set_all_btn_{"Set All Materials..."};
    std::function<void()> on_add_;
    std::function<void()> on_set_all_;
};

////////////////////////////////////////////////////////////////////////////////

Array<PropertyComponent*> make_material_options(
        wayverb::combined::model::scene& scene,
        const main_model::material_presets_t& presets,
        const wayverb::combined::model::
                min_size_vector<wayverb::combined::model::material, 1>& model,
        std::function<void()> on_add,
        std::function<void()> on_set_all,
        std::function<void(size_t)> on_delete = nullptr) {
    Array<PropertyComponent*> ret;
    size_t count = 0;
    for (const auto& i : model) {
        ret.add(new wrapped_property_component<
                left_bar::materials::config_item>(
                scene, presets, i.item(), count++, on_delete));
    }
    // Append material action buttons at the bottom of the section
    ret.add(new wrapped_property_component<material_buttons_component>(
            std::move(on_add), std::move(on_set_all)));
    return ret;
}

}  // namespace

namespace left_bar {

class label_property_component final : public PropertyComponent {
public:
    label_property_component(const String& name, const String& value)
            : PropertyComponent(name)
            , label_{"", value} {
        addAndMakeVisible(label_);
    }

    void refresh() override {}

private:
    Label label_;
};

class scale_property_component final : public PropertyComponent,
                                        public ComboBox::Listener {
public:
    scale_property_component(project& proj, std::function<void()> on_change)
            : PropertyComponent("Scale")
            , project_{proj}
            , on_change_{std::move(on_change)} {
        combo_.addItem("Metres (x1)", 1);
        combo_.addItem("Centimetres (x0.01)", 2);
        combo_.addItem("Millimetres (x0.001)", 3);
        combo_.addItem("Inches (x0.0254)", 4);
        combo_.addItem("Feet (x0.3048)", 5);

        // Select current scale
        const auto s = project_.get_scale();
        if (std::abs(s - 1.0f) < 1e-6f)        combo_.setSelectedId(1, dontSendNotification);
        else if (std::abs(s - 0.01f) < 1e-6f)   combo_.setSelectedId(2, dontSendNotification);
        else if (std::abs(s - 0.001f) < 1e-6f)  combo_.setSelectedId(3, dontSendNotification);
        else if (std::abs(s - 0.0254f) < 1e-5f) combo_.setSelectedId(4, dontSendNotification);
        else if (std::abs(s - 0.3048f) < 1e-4f) combo_.setSelectedId(5, dontSendNotification);
        else                                      combo_.setSelectedId(1, dontSendNotification);

        combo_.addListener(this);
        addAndMakeVisible(combo_);
    }

    void refresh() override {}

    void comboBoxChanged(ComboBox*) override {
        float new_scale = 1.0f;
        switch (combo_.getSelectedId()) {
            case 1: new_scale = 1.0f;    break;
            case 2: new_scale = 0.01f;   break;
            case 3: new_scale = 0.001f;  break;
            case 4: new_scale = 0.0254f; break;
            case 5: new_scale = 0.3048f; break;
        }
        project_.set_scale(new_scale);
        if (on_change_) on_change_();
    }

private:
    project& project_;
    ComboBox combo_;
    std::function<void()> on_change_;
};

master::master(main_model& model)
        : model_{model}
        , begun_connection_{model_.connect_begun([this] {
            property_panel_.setEnabled(false);
            bottom_.set_state(bottom::state::rendering);
        })}
        , engine_state_connection_{model_.connect_engine_state(
                  [this](auto run, auto runs, auto state, auto progress) {
                      std::string convergence;
                      if (state == wayverb::combined::state::running_raytracer
                          && progress > 0.01) {
                          //  Monte Carlo error ∝ 1/sqrt(N).
                          //  Show relative convergence as a percentage.
                          const auto conv = 100.0 * (1.0 - 1.0 / std::sqrt(
                              std::max(1.0, progress * 100000.0)));
                          char buf[32];
                          snprintf(buf, sizeof(buf), " [%.0f%% conv]", conv);
                          convergence = buf;
                      }
                      bottom_.set_bar_text(util::build_string(
                              "Run ",
                              run + 1,
                              " / ",
                              runs,
                              ": ",
                              wayverb::combined::to_string(state),
                              convergence));
                      bottom_.set_progress(progress);
                  })}
        , finished_connection_{model_.connect_finished([this] {
            property_panel_.setEnabled(true);
            bottom_.set_state(bottom::state::idle);
        })} {
    const auto aabb = wayverb::core::geo::compute_aabb(
            model_.project.get_scene_data().get_vertices());

    const auto dim = dimensions(aabb);
    const auto dim_string = util::build_string(dim.x, 'x', dim.y, 'x', dim.z);

    const auto volume = wayverb::core::estimate_room_volume(
            model_.project.get_scene_data());
    const auto volume_string = util::build_string(volume);

    auto scale_change_handler = [this] {
        const auto sd = model_.project.get_scene_data();
        const auto aabb = wayverb::core::geo::compute_aabb(
                sd.get_vertices());
        const auto c = centre(aabb);
        (*model_.project.persistent.sources())[0]->set_position(c);
        (*model_.project.persistent.receivers())[0]->set_position(c);
        const auto vol = wayverb::core::estimate_room_volume(sd);
        model_.project.persistent.raytracer()->set_room_volume(vol);
        model_.reset_view();
        refresh_materials();
    };

    property_panel_.addSection(
            "Info",
            {static_cast<PropertyComponent*>(
                     new label_property_component{"Dimensions (m)", dim_string}),
             static_cast<PropertyComponent*>(new label_property_component{
                     CharPointer_UTF8{"Volume (m\xc2\xb3)"},
                     volume_string}),
             static_cast<PropertyComponent*>(
                     new scale_property_component{model_.project,
                                                  scale_change_handler})});

    property_panel_.addSection(
            "Sources",
            {new wrapped_property_component<sources::master>{
                    aabb, *model_.project.persistent.sources()}});
    property_panel_.addSection(
            "Receivers",
            {new wrapped_property_component<receivers::master>{
                    model_.capsule_presets,
                    aabb,
                    *model_.project.persistent.receivers()}});
    property_panel_.addSection(
            "Materials",
            make_material_options(model_.scene,
                                  model_.material_presets,
                                  *model_.project.persistent.materials(),
                                  [this] { do_add_material(); },
                                  [this] { do_set_all_materials(); },
                                  [this](size_t idx) {
                                      model_.project.remove_surface(idx);
                                      refresh_materials();
                                  }),
            true /* open by default */);

    property_panel_.addSection(
            "Raytracer",
            {static_cast<PropertyComponent*>(new raytracer::quality_property{
                     *model_.project.persistent.raytracer()}),
             static_cast<PropertyComponent*>(
                     new raytracer::ray_count_property{
                             *model_.project.persistent.raytracer()}),
             static_cast<PropertyComponent*>(
                     new raytracer::img_src_order_property{
                             *model_.project.persistent.raytracer()})});

    property_panel_.addSection(
            "Waveguide",
            {new wrapped_property_component<waveguide::master>{
                    *model_.project.persistent.waveguide()}});

    property_panel_.setOpaque(false);

    addAndMakeVisible(property_panel_);
    addAndMakeVisible(bottom_);

    // Auto-refresh materials panel when surfaces are added or reassigned
    model_.project.on_surfaces_changed = [this] {
        MessageManager::callAsync([this] { refresh_materials(); });
    };
}

void master::resized() {
    const auto bottom_height = 30;
    auto bounds = getLocalBounds();
    bottom_.setBounds(bounds.removeFromBottom(bottom_height));
    property_panel_.setBounds(bounds);
}

void master::do_add_material() {
    const auto& mats = *model_.project.persistent.materials();
    auto name = "Material " + std::to_string(mats.size() + 1);
    model_.project.add_surface(name);
    refresh_materials();
}

void master::do_set_all_materials() {
    PopupMenu menu;
    const auto& presets = model_.material_presets;
    for (int i = 0; i < int(presets.size()); ++i) {
        menu.addItem(i + 1, presets[i].get_name());
    }
    const int result = menu.show();
    if (result > 0) {
        const auto& surface = presets[result - 1].get_surface();
        auto& mats = *model_.project.persistent.materials();
        for (auto& mat : mats) {
            mat.item()->set_surface(surface);
        }
        refresh_materials();
    }
}

void master::refresh_materials() {
    property_panel_.clear();

    const auto aabb = wayverb::core::geo::compute_aabb(
            model_.project.get_scene_data().get_vertices());
    const auto dim = dimensions(aabb);
    const auto dim_string = util::build_string(dim.x, 'x', dim.y, 'x', dim.z);
    const auto volume = wayverb::core::estimate_room_volume(
            model_.project.get_scene_data());
    const auto volume_string = util::build_string(volume);

    auto scale_change_handler = [this] {
        const auto sd = model_.project.get_scene_data();
        const auto aabb = wayverb::core::geo::compute_aabb(
                sd.get_vertices());
        const auto c = centre(aabb);
        (*model_.project.persistent.sources())[0]->set_position(c);
        (*model_.project.persistent.receivers())[0]->set_position(c);
        const auto vol = wayverb::core::estimate_room_volume(sd);
        model_.project.persistent.raytracer()->set_room_volume(vol);
        model_.reset_view();
        refresh_materials();
    };

    property_panel_.addSection(
            "Info",
            {static_cast<PropertyComponent*>(
                     new label_property_component{"Dimensions (m)", dim_string}),
             static_cast<PropertyComponent*>(new label_property_component{
                     CharPointer_UTF8{"Volume (m\xc2\xb3)"},
                     volume_string}),
             static_cast<PropertyComponent*>(
                     new scale_property_component{model_.project,
                                                  scale_change_handler})});
    property_panel_.addSection(
            "Sources",
            {new wrapped_property_component<sources::master>{
                    aabb, *model_.project.persistent.sources()}});
    property_panel_.addSection(
            "Receivers",
            {new wrapped_property_component<receivers::master>{
                    model_.capsule_presets,
                    aabb,
                    *model_.project.persistent.receivers()}});
    property_panel_.addSection(
            "Materials",
            make_material_options(model_.scene,
                                  model_.material_presets,
                                  *model_.project.persistent.materials(),
                                  [this] { do_add_material(); },
                                  [this] { do_set_all_materials(); },
                                  [this](size_t idx) {
                                      model_.project.remove_surface(idx);
                                      refresh_materials();
                                  }));
    property_panel_.addSection(
            "Raytracer",
            {static_cast<PropertyComponent*>(new raytracer::quality_property{
                     *model_.project.persistent.raytracer()}),
             static_cast<PropertyComponent*>(
                     new raytracer::ray_count_property{
                             *model_.project.persistent.raytracer()}),
             static_cast<PropertyComponent*>(
                     new raytracer::img_src_order_property{
                             *model_.project.persistent.raytracer()})});
    property_panel_.addSection(
            "Waveguide",
            {new wrapped_property_component<waveguide::master>{
                    *model_.project.persistent.waveguide()}});
}

}  // namespace left_bar
