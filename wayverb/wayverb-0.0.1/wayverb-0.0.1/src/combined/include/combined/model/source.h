#pragma once

#include "combined/model/hover.h"
#include "combined/model/min_size_vector.h"

#include "core/geo/box.h"
#include "core/orientation.h"

#include "cereal/types/base_class.hpp"

#include <string>
#include <cmath>

namespace wayverb {
namespace combined {
namespace model {

/// Source directivity pattern.
/// Determines how energy is distributed across emission angles.
enum class directivity_pattern {
    omnidirectional = 0,  ///< Equal radiation in all directions.
    cardioid,             ///< 0.5 × (1 + cos θ)
    supercardioid,        ///< 0.37 + 0.63 × cos θ
    hypercardioid,        ///< 0.25 + 0.75 × cos θ
    figure_eight,         ///< |cos θ|
    hemisphere,           ///< cos θ for θ < 90°, else 0
};

/// Compute the directivity gain for a given angle from the source's forward
/// axis.  Returns a linear gain factor [0, 1].
inline float directivity_gain(directivity_pattern pattern, float cos_theta) {
    switch (pattern) {
        case directivity_pattern::omnidirectional:
            return 1.0f;
        case directivity_pattern::cardioid:
            return std::max(0.0f, 0.5f * (1.0f + cos_theta));
        case directivity_pattern::supercardioid:
            return std::max(0.0f, 0.37f + 0.63f * cos_theta);
        case directivity_pattern::hypercardioid:
            return std::max(0.0f, 0.25f + 0.75f * cos_theta);
        case directivity_pattern::figure_eight:
            return std::abs(cos_theta);
        case directivity_pattern::hemisphere:
            return cos_theta > 0.0f ? cos_theta : 0.0f;
        default:
            return 1.0f;
    }
}

class source final : public owning_member<source, hover_state> {
public:
    explicit source(std::string name = "new source",
                    glm::vec3 position = glm::vec3{0});

    void set_name(std::string name);
    std::string get_name() const;

    void set_position(const glm::vec3& position);
    glm::vec3 get_position() const;

    void set_orientation(const core::orientation& orientation);
    core::orientation get_orientation() const;

    void set_directivity(directivity_pattern pattern);
    directivity_pattern get_directivity() const;

    using hover_state_t = class hover_state;
    const auto& hover_state() const { return get<0>(); }
    auto& hover_state() { return get<0>(); }

    template <typename Archive>
    void serialize(Archive& archive) {
        archive(name_, position_);
        //  New fields — use try/catch for backward compatibility with
        //  old project files that don't have orientation/directivity.
        try { archive(orientation_); } catch (...) {}
        try {
            int d = static_cast<int>(directivity_);
            archive(d);
            directivity_ = static_cast<directivity_pattern>(d);
        } catch (...) {}
    }

    NOTIFYING_COPY_ASSIGN_DECLARATION(source)
private:
    inline void swap(source& other) noexcept {
        using std::swap;
        swap(name_, other.name_);
        swap(position_, other.position_);
        swap(orientation_, other.orientation_);
        swap(directivity_, other.directivity_);
    };

    std::string name_;
    glm::vec3 position_;
    core::orientation orientation_;
    directivity_pattern directivity_ = directivity_pattern::omnidirectional;
};

bool operator==(const source& a, const source& b);
bool operator!=(const source& a, const source& b);

////////////////////////////////////////////////////////////////////////////////

using sources = min_size_vector<source, 1>;

}  // namespace model
}  // namespace combined
}  // namespace wayverb
