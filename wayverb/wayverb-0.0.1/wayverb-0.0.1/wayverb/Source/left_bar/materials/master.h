#pragma once

#include "main_model.h"

#include "../../UtilityComponents/connector.h"
#include "../vector_list_box.h"

#include "../JuceLibraryCode/JuceHeader.h"

namespace left_bar {
namespace materials {

class config_item final : public Component,
                          public TextButton::Listener,
                          public Label::Listener {
public:
    using material_t = wayverb::combined::model::material;
    using presets_t = main_model::material_presets_t;

    using on_delete_t = std::function<void(size_t)>;

    config_item(wayverb::combined::model::scene& scene,
                const presets_t& presets,
                std::shared_ptr<material_t> model,
                size_t index,
                on_delete_t on_delete = nullptr);

    void resized() override;
    void buttonClicked(Button* b) override;
    void labelTextChanged(Label* l) override;

private:
    wayverb::combined::model::scene& scene_;
    wayverb::combined::model::scene::visible_surface_changed::scoped_connection
            scene_connection_;

    const presets_t& presets_;

    std::shared_ptr<material_t> model_;
    const size_t index_;
    on_delete_t on_delete_;

    Label label_;
    TextButton show_button_{"Show"};
    model::Connector<TextButton> show_button_connector_{&show_button_, this};

    TextButton config_button_{"..."};
    model::Connector<TextButton> config_button_connector_{&config_button_,
                                                          this};

    TextButton delete_button_{"x"};
    model::Connector<TextButton> delete_button_connector_{&delete_button_,
                                                           this};
};

}  // namespace materials
}  // namespace left_bar
