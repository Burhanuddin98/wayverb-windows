#include "GlowSphereObject.h"

////////////////////////////////////////////////////////////////////////////////
//  GlowSphereShader
////////////////////////////////////////////////////////////////////////////////

//  The vertex shader sizes the gl_PointSize so the sprite scales correctly
//  with depth.  world_radius is extracted from the model matrix column length
//  (= the uniform scale factor set via node::set_scale).
//
//  The fragment shader draws three overlapping gaussian layers via
//  gl_PointCoord, giving a bright core with a wide soft halo.

const char* GlowSphereShader::vert = R"(
#version 150
in vec3 v_position;
in vec4 v_color;
out vec4 f_color;

uniform mat4 v_model;
uniform mat4 v_view;
uniform mat4 v_projection;

void main() {
    float world_radius = length(vec3(v_model[0]));
    vec4 view_pos = v_view * v_model * vec4(v_position, 1.0);
    gl_Position  = v_projection * view_pos;
    float depth  = max(-view_pos.z, 0.001);
    gl_PointSize = clamp(world_radius * v_projection[1][1] / depth * 300.0,
                         6.0, 256.0);
    f_color = v_color;
}
)";

const char* GlowSphereShader::frag = R"(
#version 150
in  vec4 f_color;
out vec4 frag_color;

void main() {
    vec2  coord = gl_PointCoord - vec2(0.5);
    float d     = length(coord) * 2.0;
    if (d > 1.0)
        discard;

    float glow = exp(-d * d *  3.0) * 0.6
               + exp(-d * d * 15.0) * 0.9
               + exp(-d * d * 60.0) * 1.0;

    float alpha  = clamp(glow, 0.0, 1.0) * f_color.a;
    frag_color   = vec4(f_color.rgb * glow, alpha);
}
)";

GlowSphereShader::GlowSphereShader()
        : program_{mglu::program::from_sources(vert, frag)} {}

void GlowSphereShader::set_model_matrix(const glm::mat4& m) const {
    program_.set("v_model", m);
}

void GlowSphereShader::set_view_matrix(const glm::mat4& m) const {
    program_.set("v_view", m);
}

void GlowSphereShader::set_projection_matrix(const glm::mat4& m) const {
    program_.set("v_projection", m);
}

////////////////////////////////////////////////////////////////////////////////
//  GlowSphereObject
////////////////////////////////////////////////////////////////////////////////

GlowSphereObject::GlowSphereObject(
        const std::shared_ptr<GlowSphereShader>& shader,
        const glm::vec4& color)
        : body_{shader,
                util::aligned::vector<glm::vec3>{{0.0f, 0.0f, 0.0f}},
                util::aligned::vector<glm::vec4>{color},
                util::aligned::vector<GLuint>{0},
                GL_POINTS} {}

void GlowSphereObject::do_draw(const glm::mat4& model_matrix) const {
    body_.draw(model_matrix);
}

glm::mat4 GlowSphereObject::get_local_model_matrix() const {
    return get_matrix();
}
