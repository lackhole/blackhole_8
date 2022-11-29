//
// Created by YongGyu Lee on 2022/11/16.
//

#ifndef BLACKHOLE_BLACKHOLE_SOLUTION_H_
#define BLACKHOLE_BLACKHOLE_SOLUTION_H_

#include <cmath>

#include "blackhole/math.h"
#include "blackhole/matrix.h"
#include "blackhole/object.h"

namespace blackhole {

template<typename T>
class StaticBlackhole : public DrawableObject<T> {
 public:
  using object = DrawableObject<T>;
  using value_type = T;
  using point_type = typename object::point_type;
  using vector_type = typename object::vector_type;

  StaticBlackhole(const vector_type& position, value_type mass)
    : object(position), mass_(mass), b_c_(3.0 * std::sqrt(3) * mass) {}

  value_type G(value_type u, value_type b) const {
    return (u * u * (2.0 * mass_ * u - 1.0)) + (1.0 / (b * b));
  }

  value_type InvSqrtG(value_type u, value_type b) const {
    return 1.0 / std::sqrt(G(u, b));
  }

  value_type SolveG(value_type b, int binary_search_count = 20) const {
    value_type l = 0.0;
    value_type r = 1.0 / (3.0 * mass());
    value_type mid;

    for (int i = 0; i < binary_search_count; ++i) {
      mid = (l + r) / 2.0;

      if (G(mid, b) > 0.0) {
        l = mid;
      }
      else {
        r = mid;
      }
    }

    return l;
  }

  value_type mass() const { return mass_; }
  value_type b_c() const { return b_c_; }
  value_type radius() const { return 2 * mass(); }

  const point_type& center() const { return this->position(); }

  cv::Vec3b color(value_type x, value_type y, value_type z) const override {
    return {0, 0, 0};
  }

  bool Collide(const point_type& q1, const point_type& q2, point_type *intersection) const override {
    const auto p1 = q1 - center();
    const auto p2 = q2 - center();
    const auto d1 = std::sqrt(p1.dot(p1));
    const auto d2 = std::sqrt(p2.dot(p2));

    if (d1 < radius() && d2 < radius())
      return false;

    const auto Q1 = q1 - center();
    const auto Q2 = q2 - center();
    const auto Q = Q2 - Q1;

    const auto c = std::pow(Q.dot(Q1), 2) - Q.dot(Q)*(Q1.dot(Q1) - radius() * radius());
    if (c <= 0)
      return false;

    const value_type t = (((value_type) 1) / Q.dot(Q)) * (-Q.dot(Q1) - std::sqrt(c));
    if (t <= 0 || t >= 1)
      return false;

    *intersection = Q1 + Q * t + center();
    return true;
  }

 private:
  value_type mass_;
  value_type b_c_;
};

template<typename T>
using SchwarzschildBlackhole = StaticBlackhole<T>;

} // namespace blackhole

#endif // BLACKHOLE_BLACKHOLE_SOLUTION_H_
