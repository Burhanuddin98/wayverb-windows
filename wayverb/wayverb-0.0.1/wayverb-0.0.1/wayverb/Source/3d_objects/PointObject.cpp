#include "PointObject.h"

#include <algorithm>
#include <functional>
#include <numeric>

namespace {
util::aligned::vector<glm::vec3> compute_ring_points(int num,
                                                     RingObject::Axis axis) {
    util::aligned::vector<glm::vec3> ret(num);
    for (auto i = 0; i != num; ++i) {
        auto angle = M_PI * 2 * i / num;
        switch (axis) {
            case RingObject::Axis::x:
                ret[i] = glm::vec3(sin(angle), cos(angle), 0);
                break;
            case RingObject::Axis::y:
                ret[i] = glm::vec3(0, sin(angle), cos(angle));
                break;
            case RingObject::Axis::z:
                ret[i] = glm::vec3(cos(angle), 0, sin(angle));
                break;
        }
    }
    return ret;
}

/// Generate a 3D polar pattern as a solid triangle mesh (UV sphere
/// with vertex radii modulated by gain_func).
/// Forward axis is -Z (wayverb convention: pointing = {0,0,-1}).
struct polar_mesh {
    util::aligned::vector<glm::vec3> vertices;
    util::aligned::vector<glm::vec4> colors;
    util::aligned::vector<GLuint> indices;
};

/// Build solid triangles for the balloon fill.
polar_mesh compute_polar_fill(const std::function<float(float)>& gain_func,
                              const glm::vec4& color) {
    constexpr int n_lon = 32;
    constexpr int n_lat = 24;

    polar_mesh m;

    for (int lat = 0; lat <= n_lat; ++lat) {
        const auto theta = static_cast<float>(M_PI) * lat / n_lat;
        const auto cos_theta = std::cos(theta);
        const auto sin_theta = std::sin(theta);
        const auto gain_cos = -cos_theta;
        const auto r = gain_func(gain_cos);

        for (int lon = 0; lon <= n_lon; ++lon) {
            const auto phi = static_cast<float>(2.0 * M_PI) * lon / n_lon;
            m.vertices.push_back(glm::vec3(
                    r * sin_theta * std::cos(phi),
                    r * sin_theta * std::sin(phi),
                    r * cos_theta));

            // Higher alpha where gain is strong, dimmer where weak.
            const auto a = 0.08f + 0.30f * r;
            m.colors.push_back(glm::vec4(color.r, color.g, color.b, a));
        }
    }

    for (int lat = 0; lat < n_lat; ++lat) {
        for (int lon = 0; lon < n_lon; ++lon) {
            const auto row0 = static_cast<GLuint>(lat * (n_lon + 1) + lon);
            const auto row1 = static_cast<GLuint>((lat + 1) * (n_lon + 1) + lon);
            m.indices.push_back(row0);
            m.indices.push_back(row1);
            m.indices.push_back(row0 + 1);
            m.indices.push_back(row0 + 1);
            m.indices.push_back(row1);
            m.indices.push_back(row1 + 1);
        }
    }
    return m;
}

/// Build wireframe lines (longitude + latitude rings) for the balloon outline.
polar_mesh compute_polar_wireframe(const std::function<float(float)>& gain_func,
                                   const glm::vec4& color) {
    constexpr int n_lon_slices = 8;  // longitude planes
    constexpr int n_pts = 48;         // points per slice

    polar_mesh m;
    const auto wire_color = glm::vec4(color.r, color.g, color.b, 0.7f);

    // Longitude slices.
    for (int s = 0; s < n_lon_slices; ++s) {
        const auto phi = static_cast<float>(M_PI) * s / n_lon_slices;
        const auto base = static_cast<GLuint>(m.vertices.size());
        for (int i = 0; i <= n_pts; ++i) {
            const auto theta = static_cast<float>(M_PI) * 2.0f * i / n_pts;
            const auto cos_t = std::cos(theta);
            const auto sin_t = std::sin(theta);
            // Gain from angle to -Z: cos_angle = -cos_theta when in XZ plane
            // For a general longitude slice at phi, the point direction is:
            //   (sin_t * cos_phi, sin_t * sin_phi, cos_t)
            // and forward is (0,0,-1), so cos_angle = -cos_t
            const auto r = gain_func(-cos_t);
            m.vertices.push_back(glm::vec3(
                    r * sin_t * std::cos(phi),
                    r * sin_t * std::sin(phi),
                    r * cos_t));
            m.colors.push_back(wire_color);
            m.indices.push_back(base + i);
        }
    }

    // 3 latitude rings at theta = 60, 90, 120 degrees.
    for (float theta_deg : {60.0f, 90.0f, 120.0f}) {
        const auto theta = theta_deg * static_cast<float>(M_PI) / 180.0f;
        const auto cos_t = std::cos(theta);
        const auto sin_t = std::sin(theta);
        const auto r = gain_func(-cos_t);
        const auto base = static_cast<GLuint>(m.vertices.size());
        constexpr int n_eq = 48;
        for (int i = 0; i <= n_eq; ++i) {
            const auto phi = static_cast<float>(2.0 * M_PI) * i / n_eq;
            m.vertices.push_back(glm::vec3(
                    r * sin_t * std::cos(phi),
                    r * sin_t * std::sin(phi),
                    r * cos_t));
            m.colors.push_back(wire_color);
            m.indices.push_back(base + i);
        }
    }

    return m;
}

glm::vec4 compute_ring_color(RingObject::Axis axis) {
    switch (axis) {
        case RingObject::Axis::x: return glm::vec4(1, 0, 0, 1);
        case RingObject::Axis::y: return glm::vec4(0, 1, 0, 1);
        case RingObject::Axis::z: return glm::vec4(0, 0, 1, 1);
    }
}

util::aligned::vector<GLuint> compute_ring_indices(int num) {
    util::aligned::vector<GLuint> ret(num + 1, 0);
    std::iota(ret.begin(), ret.end() - 1, 0);
    return ret;
}
}  // namespace

RingObject::RingObject(const std::shared_ptr<mglu::generic_shader>& shader,
                       const glm::vec4& color,
                       Axis axis)
        : BasicDrawableObject(shader,
                              compute_ring_points(pts, axis),
                              util::aligned::vector<glm::vec4>(pts, color),
                              compute_ring_indices(pts),
                              GL_LINE_STRIP) {}

//----------------------------------------------------------------------------//

LineObject::LineObject(const std::shared_ptr<mglu::generic_shader>& shader,
                       const glm::vec4& color)
        : BasicDrawableObject(shader,
                              {{0, 0, 0}, {0, 0, -1}},
                              util::aligned::vector<glm::vec4>(2, color),
                              {0, 1},
                              GL_LINES) {}

//----------------------------------------------------------------------------//

PolarPatternObject::PolarPatternObject(
        const std::shared_ptr<mglu::generic_shader>& shader,
        const glm::vec4& color,
        std::function<float(float)> gain_func)
        : fill_([&]() {
              auto m = compute_polar_fill(gain_func, color);
              return BasicDrawableObject(shader, m.vertices, m.colors,
                                         m.indices, GL_TRIANGLES);
          }())
        , wireframe_([&]() {
              auto m = compute_polar_wireframe(gain_func, color);
              return BasicDrawableObject(shader, m.vertices, m.colors,
                                         m.indices, GL_LINE_STRIP);
          }()) {}

glm::mat4 PolarPatternObject::get_local_model_matrix() const {
    return get_matrix();
}

void PolarPatternObject::do_draw(const glm::mat4& model_matrix) const {
    // Save GL state that we modify.
    GLboolean old_depth_mask = GL_TRUE;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &old_depth_mask);
    const auto old_cull = glIsEnabled(GL_CULL_FACE);

    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE);
    fill_.draw(model_matrix);
    wireframe_.draw(model_matrix);

    // Restore.
    if (old_cull) glEnable(GL_CULL_FACE);
    glDepthMask(old_depth_mask);
}

//----------------------------------------------------------------------------//

PointObject::PointObject(const std::shared_ptr<mglu::generic_shader>& shader,
                         const glm::vec4& color)
        : shader(shader)
        , color(color)
        , x_ring(shader, color, RingObject::Axis::x)
        , y_ring(shader, color, RingObject::Axis::y)
        , z_ring(shader, color, RingObject::Axis::z) {}

void PointObject::set_highlight(float amount) {
    x_ring.set_highlight(amount);
    y_ring.set_highlight(amount);
    z_ring.set_highlight(amount);
}

void PointObject::set_pattern(std::function<float(float)> gain_func) {
    patterns_.clear();
    if (gain_func) {
        patterns_.push_back(std::make_unique<PolarPatternObject>(
                shader, color, std::move(gain_func)));
    }
}

void PointObject::set_patterns(const std::vector<capsule_pattern>& caps) {
    patterns_.clear();
    for (const auto& cp : caps) {
        if (!cp.gain_func) continue;
        auto p = std::make_unique<PolarPatternObject>(
                shader, color, cp.gain_func);
        p->set_pointing(cp.pointing);
        patterns_.push_back(std::move(p));
    }
}

void PointObject::do_draw(const glm::mat4& model_matrix) const {
    x_ring.draw(model_matrix);
    y_ring.draw(model_matrix);
    z_ring.draw(model_matrix);
    for (auto& i : lines) {
        i.draw(model_matrix);
    }
    for (auto& p : patterns_) {
        p->draw(model_matrix);
    }
}

glm::mat4 PointObject::get_local_model_matrix() const { return get_matrix(); }

void PointObject::set_pointing(
        const util::aligned::vector<glm::vec3>& directions) {
    while (directions.size() < lines.size()) {
        lines.pop_back();
    }
    for (auto i = 0u; i != directions.size(); ++i) {
        if (lines.size() <= i) {
            lines.emplace_back(shader, color);
        }
        lines[i].set_pointing(directions[i]);
    }
}
