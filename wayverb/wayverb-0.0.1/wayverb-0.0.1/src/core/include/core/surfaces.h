#pragma once

#include "core/cl/scene_structs.h"

namespace wayverb {
namespace core {

//  for both methods:
//      define absorption coefficients per-band
//      convert to absolute pressure-reflectance values 
//          should really be complex *shrug*
//      
//  waveguide:
//      generate reflectance filter from pressure-reflectance per-band values
//      convert to impedance filter
//
//  image source:
//      generate wall impedances from pressure reflectances
//      convert back taking angle into account
//
//  scattering:
//      ???

template <typename T>
constexpr T absorption_to_energy_reflectance(T t) {
    return 1 - t;
}

template <typename T>
T absorption_to_pressure_reflectance(T t) {
    using std::sqrt;
    return sqrt(absorption_to_energy_reflectance(t));
}

template <typename T>
constexpr T pressure_reflectance_to_average_wall_impedance(T t) {
    //  Guard: if any element of (1-t) is zero, the result is huge but finite.
    return (1 + t) / (1 - t + 1e-6f);
}

template <typename T>
constexpr T average_wall_impedance_to_pressure_reflectance(T t,
                                                           float cos_angle) {
    if (cos_angle < 0 || 1 < cos_angle) {
        throw std::runtime_error{"Cos angle is outside valid range."};
    }
    const T tmp = t * cos_angle;
    const T ret = (tmp - 1) / (tmp + 1);
    return ret;
}

//  vorlander2007 p. 45
//  absorption = a, scattering = s
//  total reflected energy = 1 - a
//  scattered energy = s (1 - a)
//  specular energy = (1 - s) (1 - a)

template <typename T, typename U>
T scattered_pressure(T total_reflected, U scattering) {
    using std::sqrt;
    return total_reflected * sqrt(scattering);
}

template <typename T, typename U>
T specular_pressure(T total_reflected, U scattering) {
    using std::sqrt;
    return total_reflected * sqrt(1 - scattering);
}

}  // namespace core
}  // namespace wayverb
