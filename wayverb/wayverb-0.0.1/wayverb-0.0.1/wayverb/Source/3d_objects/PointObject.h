#pragma once

#include "BasicDrawableObject.h"

#include <functional>
#include <memory>
#include <vector>

class RingObject : public BasicDrawableObject {
public:
    enum class Axis { x, y, z };
    static constexpr auto pts = 24;
    RingObject(const std::shared_ptr<mglu::generic_shader>& shader,
               const glm::vec4& color,
               Axis axis);
};

class LineObject : public BasicDrawableObject {
public:
    LineObject(const std::shared_ptr<mglu::generic_shader>& shader,
               const glm::vec4& color);
    LineObject(LineObject&&) noexcept = default;
    LineObject& operator=(LineObject&&) noexcept = default;
};

/// 3D polar pattern — solid fill + wireframe outline.
/// Drawn with depth write disabled so the whole balloon is always visible.
class PolarPatternObject : public mglu::drawable, public node {
public:
    /// gain_func takes cos(theta) and returns gain magnitude (use abs for lobes).
    PolarPatternObject(const std::shared_ptr<mglu::generic_shader>& shader,
                       const glm::vec4& color,
                       std::function<float(float)> gain_func);
    PolarPatternObject(PolarPatternObject&&) noexcept = default;
    PolarPatternObject& operator=(PolarPatternObject&&) noexcept = default;

private:
    void do_draw(const glm::mat4& model_matrix) const override;
    glm::mat4 get_local_model_matrix() const override;

    BasicDrawableObject fill_;
    BasicDrawableObject wireframe_;
};

/// Describes one capsule's polar pattern for 3D visualization.
struct capsule_pattern {
    std::function<float(float)> gain_func;  ///< cos(theta) -> gain magnitude
    glm::vec3 pointing;                      ///< forward direction
};

class PointObject : public mglu::drawable, public node {
public:
    PointObject(const std::shared_ptr<mglu::generic_shader>& shader,
                const glm::vec4& color);

    void set_pointing(const util::aligned::vector<glm::vec3>& directions);

    /// Set one or more capsule patterns, each with its own orientation.
    void set_patterns(const std::vector<capsule_pattern>& patterns);

    /// Convenience: single pattern (used by sources).
    void set_pattern(std::function<float(float)> gain_func);

    void set_highlight(float amount);

private:
    void do_draw(const glm::mat4& model_matrix) const override;
    glm::mat4 get_local_model_matrix() const override;

    std::shared_ptr<mglu::generic_shader> shader;
    glm::vec4 color;

    RingObject x_ring;
    RingObject y_ring;
    RingObject z_ring;

    util::aligned::vector<LineObject> lines;
    std::vector<std::unique_ptr<PolarPatternObject>> patterns_;
};
