#include "master.h"
#include "../list_config_item.h"

#include "../vec3_property.h"
#include "text_property.h"

#include "combined/model/source.h"

#include <cmath>

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

/// Polar diagram showing the directivity radiation pattern.
class directivity_diagram final : public Component {
public:
    using pattern_t = wayverb::combined::model::directivity_pattern;

    void set_pattern(pattern_t p) {
        pattern_ = p;
        repaint();
    }

    void paint(Graphics& g) override {
        // Dark background with subtle border
        g.setColour(Colour(0xff1a1a2e));
        g.fillRoundedRectangle(getLocalBounds().toFloat(), 6.0f);
        g.setColour(Colour(0xff3a3a5a));
        g.drawRoundedRectangle(getLocalBounds().toFloat().reduced(0.5f), 6.0f, 1.0f);

        // Diagram area (leave bottom 20px for label)
        const auto full = getLocalBounds().toFloat().reduced(6.0f);
        const auto diag = full.withTrimmedBottom(20.0f);
        const float cx = diag.getCentreX();
        const float cy = diag.getCentreY();
        const float r  = jmin(diag.getWidth(), diag.getHeight()) * 0.46f;

        // Polar grid: 4 rings
        g.setColour(Colour(0xff252545));
        for (int i = 1; i <= 4; ++i) {
            const float ri = r * i / 4.0f;
            g.drawEllipse(cx - ri, cy - ri, ri * 2.0f, ri * 2.0f, 0.5f);
        }

        // Grid spokes every 30 degrees
        for (int deg = 0; deg < 180; deg += 30) {
            const float rad = deg * 3.14159265358979f / 180.0f;
            g.drawLine(cx - r * std::cos(rad), cy - r * std::sin(rad),
                       cx + r * std::cos(rad), cy + r * std::sin(rad), 0.5f);
        }

        // Axis labels (plain ASCII — no Unicode degree symbol)
        g.setColour(Colour(0xff777799));
        g.setFont(Font(9.0f));
        g.drawText("0",   (int)(cx - 6),      (int)(cy - r - 14), 12, 12, Justification::centred);
        g.drawText("90",  (int)(cx + r + 2),  (int)(cy - 6),      22, 12, Justification::centredLeft);
        g.drawText("180", (int)(cx - 12),     (int)(cy + r + 2),  24, 12, Justification::centred);
        g.drawText("270", (int)(cx - r - 26), (int)(cy - 6),      24, 12, Justification::centredLeft);

        // Pattern curve (θ=0 is top / front axis)
        Path path;
        for (int deg = 0; deg <= 360; ++deg) {
            const float theta = deg * 3.14159265358979f / 180.0f;
            const float mag = get_magnitude(std::cos(theta));
            const float px = cx + r * mag * std::sin(theta);
            const float py = cy - r * mag * std::cos(theta);
            if (deg == 0) path.startNewSubPath(px, py);
            else          path.lineTo(px, py);
        }
        path.closeSubPath();

        const Colour pattern_col(0xffaa44ff);
        g.setColour(pattern_col.withAlpha(0.22f));
        g.fillPath(path);
        g.setColour(pattern_col);
        g.strokePath(path, PathStrokeType(1.5f));

        // Front direction tick
        g.setColour(Colour(0xffcc66ff));
        g.drawLine(cx, cy - r - 2, cx, cy - r + 5, 1.5f);

        // Pattern name label
        g.setColour(Colours::lightgrey);
        g.setFont(Font(10.5f, Font::bold));
        g.drawText(get_label(), getLocalBounds().removeFromBottom(20),
                   Justification::centred);
    }

private:
    pattern_t pattern_{pattern_t::omnidirectional};

    /// Returns normalised magnitude [0..1] given cos(θ).
    float get_magnitude(float cos_t) const {
        using P = pattern_t;
        switch (pattern_) {
            case P::omnidirectional: return 1.0f;
            case P::cardioid:        return 0.5f + 0.5f * cos_t;
            case P::supercardioid:   return std::abs(0.37f + 0.63f * cos_t);
            case P::hypercardioid:   return std::abs(0.25f + 0.75f * cos_t);
            case P::figure_eight:    return std::abs(cos_t);
            case P::hemisphere:      return cos_t >= 0.0f ? cos_t : 0.0f;
            default: return 1.0f;
        }
    }

    const char* get_label() const {
        using P = pattern_t;
        switch (pattern_) {
            case P::omnidirectional: return "omnidirectional";
            case P::cardioid:        return "cardioid";
            case P::supercardioid:   return "supercardioid";
            case P::hypercardioid:   return "hypercardioid";
            case P::figure_eight:    return "figure-eight";
            case P::hemisphere:      return "hemisphere";
            default: return "";
        }
    }
};

////////////////////////////////////////////////////////////////////////////////

class source_editor final : public Component {
public:
    source_editor(wayverb::core::geo::box aabb,
                  std::shared_ptr<wayverb::combined::model::source> source)
            : aabb_{std::move(aabb)}
            , source_{std::move(source)} {
        //  Make properties.
        auto name        = std::make_unique<text_property>("name");
        auto position    = std::make_unique<vec3_property>("position", aabb_);
        auto directivity = std::make_unique<directivity_property>("directivity");

        auto update_from_source =
                [ this, n = name.get(), p = position.get(),
                  d = directivity.get() ](auto& source) {
            n->set(source.get_name());
            p->set(source.get_position());
            d->set(source.get_directivity());
            diagram_.set_pattern(source.get_directivity());
        };

        update_from_source(*source_);

        connection_ = wayverb::combined::model::source::scoped_connection{
                source_->connect(update_from_source)};

        name->connect_on_change(
                [this](auto&, auto name) { source_->set_name(name); });
        position->connect_on_change(
                [this](auto&, auto pos) { source_->set_position(pos); });
        directivity->connect_on_change(
                [this](auto pattern) { source_->set_directivity(pattern); });

        panel_.addProperties({name.release()});
        panel_.addProperties({position.release()});
        panel_.addProperties({directivity.release()});

        addAndMakeVisible(panel_);
        addAndMakeVisible(diagram_);

        constexpr int diagram_w = 190;
        constexpr int props_w   = 300;
        setSize(props_w + diagram_w,
                jmax(panel_.getTotalContentHeight(), diagram_w));
    }

    void resized() override {
        auto bounds = getLocalBounds();
        diagram_.setBounds(bounds.removeFromRight(190).reduced(4));
        panel_.setBounds(bounds);
    }

private:
    wayverb::core::geo::box aabb_;
    std::shared_ptr<wayverb::combined::model::source> source_;
    wayverb::combined::model::source::scoped_connection connection_;

    PropertyPanel panel_;
    directivity_diagram diagram_;
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
