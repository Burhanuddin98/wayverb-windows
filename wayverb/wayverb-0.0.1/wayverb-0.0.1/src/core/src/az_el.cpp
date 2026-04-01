#include "core/az_el.h"
#include "core/almost_equal.h"

#include "glm/gtx/transform.hpp"

#include <functional>

namespace wayverb {
namespace core {
namespace {

template <typename Op>
az_el& inplace_zip(az_el& a, const az_el& b, Op op) {
    a.azimuth = op(a.azimuth, b.azimuth);
    a.elevation = op(a.elevation, b.elevation);
    a.roll = op(a.roll, b.roll);
    return a;
}

}  // namespace

az_el& operator+=(az_el& a, const az_el& b) {
    return inplace_zip(a, b, std::plus<>());
}
az_el& operator-=(az_el& a, const az_el& b) {
    return inplace_zip(a, b, std::minus<>());
}
az_el& operator*=(az_el& a, const az_el& b) {
    return inplace_zip(a, b, std::multiplies<>());
}
az_el& operator/=(az_el& a, const az_el& b) {
    return inplace_zip(a, b, std::divides<>());
}

az_el operator+(const az_el& a, const az_el& b) {
    auto copy = a;
    return copy += b;
}
az_el operator-(const az_el& a, const az_el& b) {
    auto copy = a;
    return copy -= b;
}
az_el operator*(const az_el& a, const az_el& b) {
    auto copy = a;
    return copy *= b;
}
az_el operator/(const az_el& a, const az_el& b) {
    auto copy = a;
    return copy /= b;
}

////////////////////////////////////////////////////////////////////////////////

float compute_azimuth(const glm::vec3& pointing) {
    return std::atan2(pointing.x, -pointing.z);
}

float compute_elevation(const glm::vec3& pointing) {
    return std::asin(pointing.y);
}

az_el compute_azimuth_elevation(const glm::vec3& pointing) {
    az_el ret{compute_azimuth(pointing), compute_elevation(pointing), 0.0f};
    if (almost_equal(ret.elevation, static_cast<float>(-M_PI / 2), 10) ||
        almost_equal(ret.elevation, static_cast<float>(M_PI / 2), 10)) {
        ret.azimuth = 0;
    }
    return ret;
}

az_el compute_azimuth_elevation(const glm::vec3& pointing,
                                const glm::vec3& up) {
    auto ret = compute_azimuth_elevation(pointing);
    // Compute the expected (no-roll) up vector, then measure the roll angle.
    az_el no_roll{ret.azimuth, ret.elevation, 0.0f};
    const auto expected_up = compute_up(no_roll);
    const auto fwd = compute_pointing(no_roll);
    const auto right = glm::normalize(glm::cross(fwd, expected_up));
    // Roll = angle between expected_up and actual up, measured around fwd.
    const auto dot_up    = glm::dot(up, expected_up);
    const auto dot_right = glm::dot(up, right);
    ret.roll = std::atan2(dot_right, dot_up);
    return ret;
}

glm::vec3 compute_pointing(const az_el& azel) {
    return glm::vec3(std::sin(azel.azimuth) * std::cos(azel.elevation),
                     std::sin(azel.elevation),
                     -std::cos(azel.azimuth) * std::cos(azel.elevation));
}

glm::vec3 compute_up(const az_el& azel) {
    // Default up before roll: rotate world-up {0,1,0} by the same
    // azimuth+elevation that produced the pointing vector.
    const auto fwd = compute_pointing(azel);
    // Right = cross(fwd, world_up), then up = cross(right, fwd).
    // At the poles this degenerates, so fall back to a safe axis.
    auto world_up = glm::vec3(0, 1, 0);
    if (std::abs(azel.elevation) > static_cast<float>(M_PI / 2) - 0.01f) {
        world_up = glm::vec3(0, 0, azel.elevation > 0 ? 1 : -1);
    }
    auto right = glm::normalize(glm::cross(fwd, world_up));
    auto up    = glm::normalize(glm::cross(right, fwd));

    // Apply roll: rotate 'up' around 'fwd' by roll angle.
    if (std::abs(azel.roll) > 1e-6f) {
        const float c = std::cos(azel.roll);
        const float s = std::sin(azel.roll);
        up = glm::normalize(c * up + s * right);
    }
    return up;
}

}  // namespace core
}  // namespace wayverb
