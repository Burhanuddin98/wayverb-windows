#pragma once

#include "BasicDrawableObject.h"

#include "modern_gl_utils/drawable.h"
#include "modern_gl_utils/generic_shader.h"

#include <glm/glm.hpp>
#include <memory>

//  Shader that renders GL_POINTS as a soft radial glow billboard.
//  Interface mirrors generic_shader so it can be used with BasicDrawableObject
//  and the config_shader loop in view.cpp.

class GlowSphereShader final {
public:
    GlowSphereShader();

    auto get_attrib_location_v_position() const {
        return program_.get_attrib_location("v_position");
    }
    auto get_attrib_location_v_color() const {
        return program_.get_attrib_location("v_color");
    }

    auto get_scoped() const { return program_.get_scoped(); }

    void set_model_matrix(const glm::mat4& m) const;
    void set_view_matrix(const glm::mat4& m) const;
    void set_projection_matrix(const glm::mat4& m) const;

private:
    static const char* vert;
    static const char* frag;

    mglu::program program_;
};

////////////////////////////////////////////////////////////////////////////////

//  A drawable that shows as a glowing soft disc.
//  Uses BasicDrawableObject internally (GL_POINTS, single vertex at origin).
//  Position and scale come from the inherited node transform.

class GlowSphereObject final : public mglu::drawable, public node {
public:
    GlowSphereObject(const std::shared_ptr<GlowSphereShader>& shader,
                     const glm::vec4& color);

    GlowSphereObject(GlowSphereObject&&) noexcept = default;
    GlowSphereObject& operator=(GlowSphereObject&&) noexcept = default;

private:
    void do_draw(const glm::mat4& model_matrix) const override;
    glm::mat4 get_local_model_matrix() const override;

    BasicDrawableObject body_;
};
