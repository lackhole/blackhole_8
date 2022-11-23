//
// Created by YongGyu Lee on 2022/11/15.
//

#ifndef BLACKHOLE_RAY_TRACER_H_
#define BLACKHOLE_RAY_TRACER_H_

#include <cmath>
#include <utility>

#include "blackhole/object.h"

#include "opencv2/opencv.hpp"

namespace blackhole {

template<typename Point>
class BasicLinearRayRecurrence {
 public:
  using point_type = Point;
  BasicLinearRayRecurrence(const point_type& old, const point_type& present)
    : vec_(present - old)
  {
    if (const auto d2 = vec_.dot(vec_); d2 < 100) {
      vec_ *= 100.0 / d2;
    }
  }

  point_type operator()(const point_type& /* old */, const point_type& present) const {
    return present + vec_;
  }

 private:
  point_type vec_;
};

template<typename Point>
class FixedSingleBlackholeRayRecurrence {
 public:
  using point_type = Point;
  FixedSingleBlackholeRayRecurrence(
    const point_type& bh_pos, double mass,
    const point_type& old, const point_type& present)
    : points_(old, present) {}

 private:
  std::pair<point_type, point_type> points_;
  point_type blackhole_;
  double blackhole_mass_;
};

template<typename Point>
class RayTracer {
 public:
  using point_type = Point;
  using value_type = typename point_type::value_type;
  using recurrence_type = std::function<point_type(const point_type& old, const point_type& present)>;

  RayTracer(const point_type& old, const point_type& present)
    : point_(old, present), recurrence_(BasicLinearRayRecurrence{old, present})
  {}

  RayTracer(const point_type& old, const point_type& present,
            recurrence_type recurrence)
    : point_(old, present), recurrence_(std::move(recurrence))
  {}

  template<typename ObjManager>
  bool Prograde(ObjManager& obj_manager, unsigned char* color_dst, int step_size) {
    point_type inter;
    for (int s = 0; s < step_size; ++s) {
      if (const auto& obj = obj_manager.FindCollision(old(), present(), &inter); obj != nullptr) {
        const auto c = obj->color(inter);
        color_dst[0] = c[0];
        color_dst[1] = c[1];
        color_dst[2] = c[2];
        return true;
      }

      auto new_p = recurrence_(old(), present());
      old() = present();
      present() = std::move(new_p);
    }
    return false;
  }

  const point_type& old() const { return point_.first; }
  const point_type& present() const { return point_.second; }

 private:
  point_type& old() { return point_.first; }
  point_type& present() { return point_.second; }

  std::pair<point_type, point_type> point_; // old, present
  recurrence_type recurrence_;
};

template<typename Point>
RayTracer(const Point&, ...) -> RayTracer<Point>;

} // namespace blackhole

#endif // BLACKHOLE_RAY_TRACER_H_
