#pragma once

#include "../../generic_combo_box_property.h"
#include "../../generic_slider_property.h"

#include "combined/model/raytracer.h"

namespace left_bar {
namespace raytracer {

class ray_count_property final : public PropertyComponent,
                                  public Label::Listener {
public:
    using model_t = wayverb::combined::model::raytracer;

    ray_count_property(model_t& model);

private:
    void refresh() override {}
    void labelTextChanged(Label* label) override;
    void update_from_model();

    model_t& model_;
    Label label_;
    typename model_t::scoped_connection connection_;
};

////////////////////////////////////////////////////////////////////////////////

class quality_property final
        : public generic_slider_property<wayverb::combined::model::raytracer> {
public:
    quality_property(model_t& model);

private:
    void set_model(model_t& model, const value_t& e) override;
    value_t get_model(const model_t& model) const override;
};

////////////////////////////////////////////////////////////////////////////////

class img_src_order_property final
        : public generic_slider_property<wayverb::combined::model::raytracer> {
public:
    img_src_order_property(model_t& model);

private:
    void set_model(model_t& model, const value_t& e) override;
    value_t get_model(const model_t& model) const override;
};

}  // namespace raytracer
}  // namespace left_bar
