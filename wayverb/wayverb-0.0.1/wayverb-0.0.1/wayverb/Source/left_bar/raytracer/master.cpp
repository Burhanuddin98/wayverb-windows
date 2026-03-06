#include "master.h"

#include "utilities/string_builder.h"

#include <iomanip>

namespace left_bar {
namespace raytracer {

ray_count_property::ray_count_property(model_t& model)
        : PropertyComponent{"Rays", 25}
        , model_{model}
        , connection_{model_.connect(
                  [this](auto&) { this->update_from_model(); })} {
    label_.setEditable(false, true, false);
    label_.addListener(this);
    addAndMakeVisible(label_);
    update_from_model();
}

void ray_count_property::labelTextChanged(Label* label) {
    const auto text = label->getText().trim();
    if (text.isEmpty()) {
        model_.set_ray_count_override(0);
        return;
    }
    const auto val = text.getLargeIntValue();
    if (val > 0) {
        model_.set_ray_count_override(static_cast<size_t>(val));
    } else {
        model_.set_ray_count_override(0);
    }
}

void ray_count_property::update_from_model() {
    const auto override_val = model_.get_ray_count_override();
    const auto computed = model_.get().rays;
    if (override_val > 0) {
        label_.setText(String(override_val), dontSendNotification);
        label_.setColour(Label::textColourId, Colours::white);
    } else {
        label_.setText(String(computed) + " (auto)",
                       dontSendNotification);
        label_.setColour(Label::textColourId, Colours::grey);
    }
}

////////////////////////////////////////////////////////////////////////////////

quality_property::quality_property(model_t& model)
        : generic_slider_property{model, "Quality", 1, 20, 1} {
    update_from_model();
}

void quality_property::set_model(model_t& model, const value_t& e) {
    model.set_quality(e);
}

quality_property::value_t quality_property::get_model(
        const model_t& model) const {
    return model.get_quality();
}

////////////////////////////////////////////////////////////////////////////////

img_src_order_property::img_src_order_property(model_t& model)
        : generic_slider_property{model, "Image Source Levels", 0, 10, 1} {
    update_from_model();
}

void img_src_order_property::set_model(model_t& model, const value_t& e) {
    model.set_max_img_src_order(e);
}

img_src_order_property::value_t img_src_order_property::get_model(
        const model_t& model) const {
    return model.get_max_img_src_order();
}

}  // namespace raytracer
}  // namespace left_bar
