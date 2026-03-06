#include "combined/model/raytracer.h"

#include "raytracer/hit_rate.h"

namespace wayverb {
namespace combined {
namespace model {

raytracer::raytracer(double room_volume,
                     double speed_of_sound,
                     size_t quality,
                     size_t img_src_order,
                     double receiver_radius,
                     double histogram_sample_rate)
        : room_volume_{room_volume}
        , speed_of_sound_{speed_of_sound}
        , quality_{quality}
        , img_src_order_{img_src_order}
        , receiver_radius_{receiver_radius}
        , histogram_sample_rate_{histogram_sample_rate} {}

void raytracer::set_quality(size_t quality) {
    quality_ = quality;
    notify();
}

size_t raytracer::get_quality() const { return quality_; }

void raytracer::set_max_img_src_order(size_t i) {
    img_src_order_ = i;
    notify();
}

size_t raytracer::get_max_img_src_order() const { return img_src_order_; }

void raytracer::set_room_volume(double volume) {
    room_volume_ = volume;
    notify();
}

void raytracer::set_ray_count_override(size_t rays) {
    ray_count_override_ = rays;
    notify();
}

size_t raytracer::get_ray_count_override() const { return ray_count_override_; }

wayverb::raytracer::simulation_parameters raytracer::get() const {
    auto params = wayverb::raytracer::make_simulation_parameters(
            quality_,
            receiver_radius_,
            speed_of_sound_,
            histogram_sample_rate_,
            room_volume_,
            img_src_order_);
    if (ray_count_override_ > 0) {
        params.rays = ray_count_override_;
    }
    return params;
}

bool operator==(const raytracer& a, const raytracer& b) {
    return a.get() == b.get();
}

bool operator!=(const raytracer& a, const raytracer& b) { return !(a == b); }

}  // namespace model
}  // namespace combined
}  // namespace wayverb
