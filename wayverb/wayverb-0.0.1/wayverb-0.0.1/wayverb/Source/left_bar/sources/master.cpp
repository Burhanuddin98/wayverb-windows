#include "master.h"
#include "../list_config_item.h"

#include "../vec3_property.h"
#include "text_property.h"

#include "combined/model/source.h"

namespace left_bar {
namespace sources {

////////////////////////////////////////////////////////////////////////////////

/// Simple combo box property for source directivity pattern selection.
class directivity_property final : public PropertyComponent,
                                    public ComboBox::Listener {
public:
    using pattern_t = wayverb::combined::model::directivity_pattern;
    using on_change_t = std::function<void(pattern_t)>;

    directivity_property(const String& name)
            : PropertyComponent(name, 25) {
        combo_.addItem("omnidirectional", 1 + static_cast<int>(pattern_t::omnidirectional));
        combo_.addItem("cardioid",       1 + static_cast<int>(pattern_t::cardioid));
        combo_.addItem("supercardioid",  1 + static_cast<int>(pattern_t::supercardioid));
        combo_.addItem("hypercardioid",  1 + static_cast<int>(pattern_t::hypercardioid));
        combo_.addItem("figure-eight",   1 + static_cast<int>(pattern_t::figure_eight));
        combo_.addItem("hemisphere",     1 + static_cast<int>(pattern_t::hemisphere));
        combo_.addListener(this);
        addAndMakeVisible(combo_);
    }

    void set(pattern_t p) {
        combo_.setSelectedId(1 + static_cast<int>(p), dontSendNotification);
    }

    void connect_on_change(on_change_t cb) { on_change_ = std::move(cb); }

    void refresh() override {}

    void comboBoxChanged(ComboBox* cb) override {
        if (on_change_) {
            on_change_(static_cast<pattern_t>(cb->getSelectedId() - 1));
        }
    }

private:
    ComboBox combo_;
    on_change_t on_change_;
};

////////////////////////////////////////////////////////////////////////////////

class source_editor final : public PropertyPanel {
public:
    source_editor(wayverb::core::geo::box aabb,
                  std::shared_ptr<wayverb::combined::model::source> source)
            : aabb_{std::move(aabb)}
            , source_{std::move(source)} {
        //  Make properties.
        auto name = std::make_unique<text_property>("name");
        auto position = std::make_unique<vec3_property>("position", aabb_);
        auto directivity = std::make_unique<directivity_property>("directivity");

        auto update_from_source =
                [ this, n = name.get(), p = position.get(),
                  d = directivity.get() ](auto& source) {
            n->set(source.get_name());
            p->set(source.get_position());
            d->set(source.get_directivity());
        };

        update_from_source(*source_);

        //  Tell UI objects what to do when the data source changes.
        connection_ = wayverb::combined::model::source::scoped_connection{
                source_->connect(update_from_source)};

        //  Tell model what to do when the ui is updated by the user.
        name->connect_on_change(
                [this](auto&, auto name) { source_->set_name(name); });

        position->connect_on_change(
                [this](auto&, auto pos) { source_->set_position(pos); });

        directivity->connect_on_change(
                [this](auto pattern) { source_->set_directivity(pattern); });

        addProperties({name.release()});
        addProperties({position.release()});
        addProperties({directivity.release()});

        setSize(300, getTotalContentHeight());
    }

private:
    wayverb::core::geo::box aabb_;
    std::shared_ptr<wayverb::combined::model::source> source_;
    wayverb::combined::model::source::scoped_connection connection_;
};

////////////////////////////////////////////////////////////////////////////////

master::master(wayverb::core::geo::box aabb,
               wayverb::combined::model::sources& model)
        : list_box_{model,
                    [aabb](auto shared) {
                        return make_list_config_item_ptr(
                                shared, [aabb](auto shared) {
                                    return std::make_unique<source_editor>(
                                            aabb, shared);
                                },
                                "source");
                    },
                    [aabb](auto& model) {
                        wayverb::combined::model::source to_insert{};
                        to_insert.set_position(centre(aabb));
                        model.insert(model.end(), to_insert);
                    }} {
    list_box_.setRowHeight(30);
    addAndMakeVisible(list_box_);

    setSize(300, 100);
}

void master::resized() { list_box_.setBounds(getLocalBounds()); }

}  // namespace sources
}  // namespace left_bar
