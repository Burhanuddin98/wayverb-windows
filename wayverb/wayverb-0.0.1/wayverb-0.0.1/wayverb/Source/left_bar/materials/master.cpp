#include "master.h"

#include "../../generic_property_component.h"
#include "../../UtilityComponents/modal_dialog.h"

#include "utilities/string_builder.h"

#include <iomanip>

namespace left_bar {
namespace materials {

class bands_component final : public Component, public Slider::Listener {
public:
    bands_component(double min, double max, double inc) {
        for (auto& i : sliders_) {
            i.setSliderStyle(Slider::SliderStyle::LinearVertical);
            i.setTextBoxStyle(
                    Slider::TextEntryBoxPosition::TextBoxBelow, false, 40, 20);
            i.setRange(min, max, inc);
            i.addListener(this);
            addAndMakeVisible(i);
        }
    }

    void set(const wayverb::core::bands_type& bands) {
        for (auto i = 0; i != wayverb::core::simulation_bands; ++i) {
            sliders_[i].setValue(bands.s[i], dontSendNotification);
        }
    }

    wayverb::core::bands_type get() const {
        wayverb::core::bands_type ret;
        for (auto i = 0; i != wayverb::core::simulation_bands; ++i) {
            ret.s[i] = sliders_[i].getValue();
        }
        return ret;
    }

    void resized() override {
        const auto bounds = getLocalBounds();

        auto total_width = [this](auto i) {
            return i * getWidth() / sliders_.size();
        };

        const auto width = total_width(1);

        for (auto i = 0u; i != sliders_.size(); ++i) {
            sliders_[i].setBounds(total_width(i), 0, width, getHeight());
        }
    }

    void sliderValueChanged(Slider*) override { on_change_(get()); }

    using on_change = util::event<wayverb::core::bands_type>;
    on_change::connection connect_on_change(on_change::callback_type callback) {
        return on_change_.connect(std::move(callback));
    }

private:
    std::array<Slider, wayverb::core::simulation_bands> sliders_;

    on_change on_change_;
};

////////////////////////////////////////////////////////////////////////////////

class frequency_labels : public Component {
public:
    frequency_labels() {
        const auto centres = hrtf_data::hrtf_band_centres_hz();

        for (auto i = 0; i != wayverb::core::simulation_bands; ++i) {
            const auto freq = centres[i];
            if (1000 <= freq) {
                labels_[i].setText(
                        util::build_string(
                                std::setprecision(3), centres[i] / 1000, "K"),
                        dontSendNotification);
            } else {
                labels_[i].setText(
                        util::build_string(std::setprecision(3), centres[i]),
                        dontSendNotification);
            }
        }

        for (auto& i : labels_) {
            addAndMakeVisible(i);
        }
    }

    void resized() override {
        const auto bounds = getLocalBounds();

        auto total_width = [this](auto i) {
            return i * getWidth() / labels_.size();
        };

        const auto width = total_width(1);

        for (auto i = 0u; i != labels_.size(); ++i) {
            labels_[i].setBounds(total_width(i), 0, width, getHeight());
        }
    }

private:
    std::array<Label, wayverb::core::simulation_bands> labels_;
};

////////////////////////////////////////////////////////////////////////////////

class preset_list_component final : public Component, public ListBoxModel {
public:
    using material_t = wayverb::combined::model::material;
    using presets_t = main_model::material_presets_t;

    preset_list_component(const presets_t& presets, material_t& model)
            : presets_{presets}, model_{model} {
        list_box_.setModel(this);
        list_box_.setRowHeight(22);
        list_box_.setColour(ListBox::backgroundColourId, Colours::darkgrey);
        list_box_.setColour(ListBox::outlineColourId, Colours::grey);
        addAndMakeVisible(list_box_);
    }

    int getNumRows() override { return int(presets_.size()); }

    void paintListBoxItem(int row, Graphics& g, int w, int h,
                          bool selected) override {
        if (selected) g.fillAll(Colour(0xff8a2be2).withAlpha(0.4f));
        g.setColour(Colours::lightgrey);
        g.setFont(12.0f);
        if (row >= 0 && row < int(presets_.size()))
            g.drawText(presets_[row].get_name(), 6, 0, w - 6, h,
                       Justification::centredLeft);
    }

    void listBoxItemClicked(int row, const MouseEvent&) override {
        if (row >= 0 && row < int(presets_.size()))
            model_.set_surface(presets_[row].get_surface());
    }

    void resized() override { list_box_.setBounds(getLocalBounds()); }

private:
    const presets_t& presets_;
    material_t& model_;
    ListBox list_box_;
};

class material_component final : public Component {
public:
    using material_t = wayverb::combined::model::material;
    using presets_t = main_model::material_presets_t;

    material_component(const presets_t& presets, material_t& model)
            : presets_{presets}
            , model_{model}
            , preset_list_{presets, model} {
        // Left side: sliders panel
        auto frequencies =
                std::make_unique<property_component_adapter<frequency_labels>>(
                        "Band Centres (Hz)", 25);
        auto absorption =
                std::make_unique<property_component_adapter<bands_component>>(
                        "Absorption", 100, 0.01, 1.0, 0.01);
        auto scattering =
                std::make_unique<property_component_adapter<bands_component>>(
                        "Scattering", 100, 0.0, 1.0, 0.01);

        const auto update_from_material =
                [ this, a = &absorption->content, s = &scattering->content ](
                        auto& material) {
            a->set(material.get_surface().absorption);
            s->set(material.get_surface().scattering);
        };

        update_from_material(model_);

        connection_ = material_t::scoped_connection{
                model_.connect(update_from_material)};

        const auto update_from_controls =
                [ this, a = &absorption->content, s = &scattering->content ](
                        auto) {
            model_.set_surface(
                    wayverb::core::surface<wayverb::core::simulation_bands>{
                            a->get(), s->get()});
        };

        absorption->content.connect_on_change(update_from_controls);
        scattering->content.connect_on_change(update_from_controls);

        sliders_panel_.addProperties({frequencies.release()});
        sliders_panel_.addProperties({absorption.release()});
        sliders_panel_.addProperties({scattering.release()});

        addAndMakeVisible(sliders_panel_);
        addAndMakeVisible(preset_list_);
        addAndMakeVisible(preset_label_);
        preset_label_.setFont(Font(12.0f, Font::bold));
        preset_label_.setColour(Label::textColourId, Colours::lightgrey);
        preset_label_.setText("Material Presets", dontSendNotification);

        setSize(680, sliders_panel_.getTotalContentHeight());
    }

    void resized() override {
        auto bounds = getLocalBounds();
        // Right column: preset list (200px wide)
        auto presetArea = bounds.removeFromRight(200);
        auto labelArea = presetArea.removeFromTop(20);
        preset_label_.setBounds(labelArea);
        preset_list_.setBounds(presetArea);
        // Left column: sliders
        sliders_panel_.setBounds(bounds);
    }

private:
    const presets_t& presets_;
    material_t& model_;
    material_t::scoped_connection connection_;

    PropertyPanel sliders_panel_;
    preset_list_component preset_list_;
    Label preset_label_;
};

////////////////////////////////////////////////////////////////////////////////

config_item::config_item(wayverb::combined::model::scene& scene,
                         const presets_t& presets,
                         std::shared_ptr<material_t> model,
                         size_t index,
                         on_delete_t on_delete)
        : scene_{scene}
        , scene_connection_{scene_.connect_visible_surface_changed(
                  [this](auto surf) {
                      show_button_.setToggleState(surf && *surf == index_,
                                                  dontSendNotification);
                  })}
        , presets_{presets}
        , model_{std::move(model)}
        , index_{index}
        , on_delete_{std::move(on_delete)}
        , label_{"", model_->get_name()} {
    label_.setEditable(false, true, false);  // double-click to edit
    label_.addListener(this);
    label_.setTooltip("Double-click to rename.");

    addAndMakeVisible(label_);
    addAndMakeVisible(show_button_);
    addAndMakeVisible(config_button_);
    addAndMakeVisible(delete_button_);

    show_button_.setTooltip("Display the triangles which use this material.");
    config_button_.setTooltip("Configure this material.");
    delete_button_.setTooltip("Delete this material (triangles merge into first material).");
    delete_button_.setColour(TextButton::buttonColourId, Colour(0xff882222));

    setSize(300, 30);
}

void config_item::resized() {
    const auto button_width = this->getHeight();
    auto bounds = this->getLocalBounds();
    delete_button_.setBounds(
            bounds.removeFromRight(button_width).reduced(2, 2));
    config_button_.setBounds(
            bounds.removeFromRight(button_width).reduced(2, 2));
    show_button_.setBounds(
            bounds.removeFromRight(button_width * 2).reduced(2, 2));
    label_.setBounds(bounds.reduced(2, 2));
}

void config_item::labelTextChanged(Label* l) {
    if (l == &label_) {
        model_->set_name(label_.getText().toStdString());
    }
}

void config_item::buttonClicked(Button* b) {
    if (b == &show_button_) {
        scene_.set_visible_surface(
                b->getToggleState() ? std::nullopt
                                    : std::make_optional(index_));
    } else if (b == &config_button_) {
        begin_modal_dialog(
                model_->get_name(),
                make_done_window_ptr(std::make_unique<material_component>(
                        presets_, *model_)),
                [](auto) {});
    } else if (b == &delete_button_) {
        if (on_delete_) {
            on_delete_(index_);
        }
    }
}

}  // namespace materials
}  // namespace left_bar
