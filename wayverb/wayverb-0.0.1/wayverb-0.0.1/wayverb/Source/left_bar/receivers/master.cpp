#include "master.h"

#include "capsules/master.h"

#include "../azimuth_elevation_property.h"
#include "../vec3_property.h"
#include "text_property.h"

#include "combined/model/capsule.h"
#include "combined/model/receiver.h"

#include "core/az_el.h"

#include <cmath>
#include <vector>

namespace left_bar {
namespace receivers {

////////////////////////////////////////////////////////////////////////////////

/// Top-down azimuth/elevation orientation compass for a receiver.
/// Shows a head silhouette (rotates with azimuth), capsule arrows (with names),
/// tick-marked compass ring, glow arrow, and an elevation pitch bar.
class orientation_diagram final : public Component {
public:
    void set_az_el(wayverb::core::az_el az_el) {
        az_el_ = az_el;
        repaint();
    }

    /// Each entry is one capsule in the receiver's local frame.
    /// Both vectors must have the same length.
    void set_capsules(std::vector<wayverb::core::az_el> az_els,
                      std::vector<std::string> names) {
        capsules_      = std::move(az_els);
        capsule_names_ = std::move(names);
        repaint();
    }

    void paint(Graphics& g) override {
        static constexpr float kPi = 3.14159265358979f;

        const float W = (float)getWidth();
        const float H = (float)getHeight();
        const float pad = 5.0f;

        // ── Background ────────────────────────────────────────────────────────
        g.setColour(Colour(0xff181830));
        g.fillRoundedRectangle(getLocalBounds().toFloat(), 6.0f);
        g.setColour(Colour(0xff3a3a5a));
        g.drawRoundedRectangle(getLocalBounds().toFloat().reduced(0.5f), 6.0f, 1.0f);

        // ── Title ─────────────────────────────────────────────────────────────
        const float title_h = 12.0f;
        g.setColour(Colour(0xff8888bb));
        g.setFont(Font(8.0f, Font::bold));
        g.drawText("ORIENTATION  -  TOP VIEW",
                   Rectangle<float>(pad, pad, W - 2.0f*pad, title_h),
                   Justification::centred);

        // ── Az/El readout (bottom strip) ──────────────────────────────────────
        const float az = az_el_.azimuth;
        const float az_deg = az * 180.0f / kPi;
        const float el_deg = az_el_.elevation * 180.0f / kPi;
        const float readout_h = 11.0f;
        g.setColour(Colour(0xffbbbbcc));
        g.setFont(Font(9.0f));
        g.drawText(String::formatted("az %+.0f   el %+.0f", az_deg, el_deg),
                   Rectangle<float>(pad, H - pad - readout_h, W - 2.0f*pad, readout_h),
                   Justification::centred);

        // ── Elevation pitch bar ───────────────────────────────────────────────
        const float bar_h  = 7.0f;
        const float bar_mg = 2.0f;
        const float lbl_w  = 18.0f;
        const float bar_y  = H - pad - readout_h - bar_mg - bar_h;
        const float bar_x  = pad + lbl_w;
        const float bar_w  = W - 2.0f*pad - 2.0f*lbl_w;
        const float bar_cx = bar_x + bar_w * 0.5f;

        g.setColour(Colour(0xff222240));
        g.fillRoundedRectangle(bar_x, bar_y, bar_w, bar_h, 3.0f);
        g.setColour(Colour(0xff3a3a60));
        g.drawRoundedRectangle(bar_x, bar_y, bar_w, bar_h, 3.0f, 0.8f);

        const float el_norm = az_el_.elevation / (kPi * 0.5f);  // -1..+1
        const float fill_w  = std::abs(el_norm) * bar_w * 0.5f;
        const float fill_x  = el_norm >= 0.0f ? bar_cx : bar_cx - fill_w;
        if (fill_w > 0.5f) {
            g.setColour(el_norm >= 0.0f ? Colour(0xff5588cc) : Colour(0xff8844cc));
            g.fillRoundedRectangle(fill_x, bar_y, fill_w, bar_h, 3.0f);
        }
        g.setColour(Colour(0xff5555aa));
        g.drawLine(bar_cx, bar_y, bar_cx, bar_y + bar_h, 1.0f);

        g.setColour(Colour(0xff555577));
        g.setFont(Font(7.0f));
        g.drawText("-90", Rectangle<float>(pad, bar_y, lbl_w, bar_h),
                   Justification::centredLeft);
        g.drawText("+90", Rectangle<float>(W - pad - lbl_w, bar_y, lbl_w, bar_h),
                   Justification::centredRight);

        // ── Compass area bounds ───────────────────────────────────────────────
        const float compass_top = pad + title_h + 2.0f;
        const float compass_bot = bar_y - 2.0f;
        const float cx = W * 0.5f;
        const float cy = (compass_top + compass_bot) * 0.5f;
        const float r  = jmin((W - 2.0f*pad) * 0.5f,
                               (compass_bot - compass_top) * 0.5f) * 0.76f;

        // ── Compass disc ──────────────────────────────────────────────────────
        g.setColour(Colour(0xff1c1c38));
        g.fillEllipse(cx - r, cy - r, r*2.0f, r*2.0f);

        // Inner grid rings at 33% and 67%
        g.setColour(Colour(0xff252548));
        g.drawEllipse(cx - r*0.33f, cy - r*0.33f, r*0.66f, r*0.66f, 0.5f);
        g.drawEllipse(cx - r*0.67f, cy - r*0.67f, r*1.34f, r*1.34f, 0.5f);

        // Faint crosshairs
        g.setColour(Colour(0xff252548));
        g.drawLine(cx - r, cy, cx + r, cy, 0.5f);
        g.drawLine(cx, cy - r, cx, cy + r, 0.5f);

        // ── Tick marks ────────────────────────────────────────────────────────
        for (int deg = 0; deg < 360; deg += 10) {
            const float rad  = deg * kPi / 180.0f;
            const float sinA = std::sin(rad);
            const float cosA = std::cos(rad);
            float tickLen, tickThick;
            uint32_t tickCol;
            if (deg % 90 == 0) {
                tickLen = 7.0f; tickThick = 1.1f; tickCol = 0xff5555aa;
            } else if (deg % 30 == 0) {
                tickLen = 4.5f; tickThick = 0.8f; tickCol = 0xff3a3a60;
            } else {
                tickLen = 2.0f; tickThick = 0.5f; tickCol = 0xff28284a;
            }
            const float ox = cx + r * sinA;
            const float oy = cy - r * cosA;
            const float ix = cx + (r - tickLen) * sinA;
            const float iy = cy - (r - tickLen) * cosA;
            g.setColour(Colour(tickCol));
            g.drawLine(ox, oy, ix, iy, tickThick);
        }

        // Outer ring stroke
        g.setColour(Colour(0xff4444aa));
        g.drawEllipse(cx - r, cy - r, r*2.0f, r*2.0f, 1.2f);

        // ── Cardinal labels N/S/E/W ───────────────────────────────────────────
        const float lo = r + 9.0f;
        g.setColour(Colour(0xff7777aa));
        g.setFont(Font(8.0f, Font::bold));
        g.drawText("N", (int)(cx-6),    (int)(cy-lo-9),  12, 10, Justification::centred);
        g.drawText("S", (int)(cx-6),    (int)(cy+lo),    12, 10, Justification::centred);
        g.drawText("W", (int)(cx-lo-11),(int)(cy-5),     12, 10, Justification::centred);
        g.drawText("E", (int)(cx+lo-1), (int)(cy-5),     12, 10, Justification::centred);

        // ── Head silhouette — all paths built in local coords, rotated by az ─
        const float head_r = r * 0.29f;
        const float hw     = head_r;           // half-width
        const float hh     = head_r * 0.84f;  // half-height (top view: wider than tall)

        // AffineTransform: rotate around origin by az, then translate to compass centre.
        // Local +y = screen down = "south"; local -y = "north" = facing direction at az=0.
        const auto xf = AffineTransform::rotation(az, 0.0f, 0.0f).translated(cx, cy);

        // Head oval
        Path headPath;
        headPath.addEllipse(-hw, -hh, hw*2.0f, hh*2.0f);
        g.setColour(Colour(0xff272750));
        g.fillPath(headPath, xf);
        g.setColour(Colour(0xff5555aa));
        g.strokePath(headPath, PathStrokeType(1.2f), xf);

        // Ears (D-shaped ellipses on ±x sides)
        const float ear_gap = 1.5f;
        const float ear_rw  = head_r * 0.19f;
        const float ear_rh  = hh * 0.38f;
        Path leftEar, rightEar;
        leftEar.addEllipse(-(hw + ear_gap + ear_rw*2.0f), -ear_rh, ear_rw*2.0f, ear_rh*2.0f);
        rightEar.addEllipse(hw + ear_gap,                  -ear_rh, ear_rw*2.0f, ear_rh*2.0f);
        g.setColour(Colour(0xff353565));
        g.fillPath(leftEar,  xf);
        g.fillPath(rightEar, xf);
        g.setColour(Colour(0xff5555aa));
        g.strokePath(leftEar,  PathStrokeType(0.8f), xf);
        g.strokePath(rightEar, PathStrokeType(0.8f), xf);

        // Nose triangle pointing in facing direction (local -y)
        Path nosePath;
        nosePath.startNewSubPath(-hw * 0.22f, -hh * 0.88f);
        nosePath.lineTo(hw * 0.22f,  -hh * 0.88f);
        nosePath.lineTo(0.0f,        -(hh + head_r * 0.30f));
        nosePath.closeSubPath();
        g.setColour(Colour(0xff7777cc));
        g.fillPath(nosePath, xf);

        // ── Facing direction arrow (glow + solid) ─────────────────────────────
        const float arr_s = head_r * 1.30f;
        const float arr_e = r * 0.83f;
        const float as_x = cx + arr_s * std::sin(az);
        const float as_y = cy - arr_s * std::cos(az);
        const float ae_x = cx + arr_e * std::sin(az);
        const float ae_y = cy - arr_e * std::cos(az);

        // Glow (wide, semi-transparent stroke)
        Path glowLine;
        glowLine.startNewSubPath(as_x, as_y);
        glowLine.lineTo(ae_x, ae_y);
        g.setColour(Colour(0xffcc66ff).withAlpha(0.20f));
        g.strokePath(glowLine, PathStrokeType(7.0f,
                                              PathStrokeType::curved,
                                              PathStrokeType::rounded));

        // Solid arrow
        Path facingArrow;
        facingArrow.addArrow(Line<float>(as_x, as_y, ae_x, ae_y), 2.0f, 10.0f, 8.0f);
        g.setColour(Colour(0xffcc66ff));
        g.fillPath(facingArrow);

        // ── Capsule arrows (named) ─────────────────────────────────────────────
        static const Colour kCapsuleColours[] = {
            Colour(0xff00e5ff),  // cyan
            Colour(0xffff9900),  // amber
            Colour(0xff00ff88),  // green
            Colour(0xffff44aa),  // pink
            Colour(0xffffff00),  // yellow
            Colour(0xffff5544),  // red
        };
        constexpr int kNColours = 6;

        for (int ci = 0; ci < (int)capsules_.size(); ++ci) {
            const float cap_az = az_el_.azimuth + capsules_[ci].azimuth;
            const float ca_s   = head_r * 1.38f;
            const float ca_e   = r * 0.76f;
            const float cs_x = cx + ca_s * std::sin(cap_az);
            const float cs_y = cy - ca_s * std::cos(cap_az);
            const float ce_x = cx + ca_e * std::sin(cap_az);
            const float ce_y = cy - ca_e * std::cos(cap_az);

            const Colour col = kCapsuleColours[ci % kNColours];

            Path capArrow;
            capArrow.addArrow(Line<float>(cs_x, cs_y, ce_x, ce_y), 1.5f, 7.0f, 6.0f);
            g.setColour(col.withAlpha(0.88f));
            g.fillPath(capArrow);

            // Label: use actual capsule name (up to 5 chars), else fall back to number
            String lbl;
            if (ci < (int)capsule_names_.size() && !capsule_names_[ci].empty())
                lbl = String(capsule_names_[ci].c_str()).substring(0, 5).trim();
            else
                lbl = String(ci + 1);

            const float lx2 = cx + (ca_e + 11.0f) * std::sin(cap_az);
            const float ly2 = cy - (ca_e + 11.0f) * std::cos(cap_az);
            g.setColour(col);
            g.setFont(Font(7.5f, Font::bold));
            g.drawText(lbl, (int)(lx2 - 16), (int)(ly2 - 7), 32, 14, Justification::centred);
        }
    }

private:
    wayverb::core::az_el            az_el_{};
    std::vector<wayverb::core::az_el> capsules_;
    std::vector<std::string>          capsule_names_;
};

////////////////////////////////////////////////////////////////////////////////

class receiver_properties final : public PropertyPanel {
public:
    /// Called whenever orientation changes (from UI or model). Set by receiver_editor.
    std::function<void(wayverb::core::az_el)> on_orientation_updated;

    receiver_properties(wayverb::core::geo::box aabb,
                        wayverb::combined::model::receiver& receiver)
            : aabb_{std::move(aabb)}
            , receiver_{receiver} {
        auto name = std::make_unique<text_property>("name");
        auto position = std::make_unique<vec3_property>("position", aabb_);
        auto orientation =
                std::make_unique<azimuth_elevation_property>("orientation");

        const auto update_from_receiver = [
            this,
            n = name.get(),
            p = position.get(),
            o = orientation.get()
        ](auto& receiver) {
            const auto q = receiver.get_orientation().get_pointing();
            const auto az_el = wayverb::core::compute_azimuth_elevation(q);
            n->set(receiver.get_name());
            p->set(receiver.get_position());
            o->set(az_el);
            if (on_orientation_updated) on_orientation_updated(az_el);
        };

        update_from_receiver(receiver_);

        connection_ = wayverb::combined::model::receiver::scoped_connection{
                receiver_.connect(update_from_receiver)};

        name->connect_on_change(
                [this](auto&, auto name) { receiver_.set_name(name); });

        position->connect_on_change(
                [this](auto&, auto pos) { receiver_.set_position(pos); });

        orientation->connect_on_change([this](auto&, auto az_el) {
            receiver_.set_orientation(
                    wayverb::core::orientation{compute_pointing(az_el)});
            // Also update diagram immediately (before model round-trips back)
            if (on_orientation_updated) on_orientation_updated(az_el);
        });

        addProperties({name.release()});
        addProperties({position.release()});
        addProperties({orientation.release()});
    }

private:
    wayverb::core::geo::box aabb_;
    wayverb::combined::model::receiver& receiver_;
    wayverb::combined::model::receiver::scoped_connection connection_;
};

////////////////////////////////////////////////////////////////////////////////

class receiver_editor final : public Component, public ComboBox::Listener {
public:
    receiver_editor(
            const main_model::capsule_presets_t& presets,
            wayverb::core::geo::box aabb,
            std::shared_ptr<wayverb::combined::model::receiver> receiver)
            : receiver_{std::move(receiver)}
            , properties_{std::move(aabb), *receiver_}
            , capsules_{*receiver_->capsules()}
            , presets_{presets} {
        combo_box_.setTextWhenNothingSelected("capsule group presets...");

        {
            auto count = 1;
            for (const auto& i : presets_) {
                combo_box_.addItem(i.name, count++);
            }
        }

        // Wire diagram updates directly from the properties callback chain.
        properties_.on_orientation_updated = [this](wayverb::core::az_el az_el) {
            orientation_.set_az_el(az_el);
        };

        // Populate capsule arrows for initial capsule set.
        update_capsule_diagram();

        addAndMakeVisible(properties_);
        addAndMakeVisible(capsules_);
        addAndMakeVisible(combo_box_);
        addAndMakeVisible(orientation_);

        setSize(700, jmax(properties_.getTotalContentHeight(), 200));
    }

    void resized() override {
        auto bounds = getLocalBounds();
        orientation_.setBounds(bounds.removeFromRight(200).reduced(4));
        auto capsule_bounds = bounds.removeFromRight(150);
        properties_.setBounds(bounds);
        combo_box_.setBounds(capsule_bounds.removeFromBottom(25));
        capsules_.setBounds(capsule_bounds);
    }

    void comboBoxChanged(ComboBox* cb) override {
        const auto selected = cb->getSelectedItemIndex();
        if (selected != -1) {
            const auto& capsules = presets_[selected].capsules;
            receiver_->capsules()->clear();
            for (const auto& capsule : capsules) {
                receiver_->capsules()->insert(receiver_->capsules()->end(),
                                              capsule);
            }
            update_capsule_diagram();
        }
        cb->setSelectedItemIndex(-1, dontSendNotification);
    }

private:
    std::shared_ptr<wayverb::combined::model::receiver> receiver_;

    receiver_properties properties_;
    capsules::master    capsules_;
    ComboBox            combo_box_;
    orientation_diagram orientation_;

    model::Connector<ComboBox> capsule_presets_connector_{&combo_box_, this};

    main_model::capsule_presets_t presets_;

    /// Read current capsule orientations and names from the receiver model and push to diagram.
    void update_capsule_diagram() {
        std::vector<wayverb::core::az_el> az_els;
        std::vector<std::string> names;
        const auto& caps = *receiver_->capsules();
        for (auto it = caps.begin(); it != caps.end(); ++it) {
            const auto& cap = **it;
            const auto orient = wayverb::combined::model::get_orientation(cap);
            az_els.push_back(
                    wayverb::core::compute_azimuth_elevation(orient.get_pointing()));
            names.push_back(cap.get_name());
        }
        orientation_.set_capsules(std::move(az_els), std::move(names));
    }
};

////////////////////////////////////////////////////////////////////////////////

master::master(const main_model::capsule_presets_t& presets,
               wayverb::core::geo::box aabb,
               wayverb::combined::model::receivers& receivers)
        : list_box_{receivers,
                    [&presets, aabb](auto shared) {
                        return make_list_config_item_ptr(
                                shared, [&presets, aabb](auto shared) {
                                    return std::make_unique<receiver_editor>(
                                            presets, aabb, shared);
                                }, "receiver");
                    },
                    [aabb](auto& model) {
                        wayverb::combined::model::receiver to_insert{};
                        to_insert.set_position(centre(aabb));
                        model.insert(model.end(), to_insert);
                    }} {
    list_box_.setRowHeight(30);
    addAndMakeVisible(list_box_);

    setSize(300, 100);
}

void master::resized() { list_box_.setBounds(getLocalBounds()); }

}  // namespace receivers
}  // namespace left_bar
