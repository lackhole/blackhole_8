//
// Created by YongGyu Lee on 2022/11/12.
//

#ifndef BLACKHOLE_CAMERA_H_
#define BLACKHOLE_CAMERA_H_

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>

#include "opencv2/opencv.hpp"

#include "blackhole/matrix.h"
#include "blackhole/constants.h"
#include "blackhole/object.h"

namespace blackhole {

template<typename T>
class Camera : public Object<T> {
 public:
  using base = Object<T>;
  using value_type = typename base::value_type;
  using point_type = typename base::point_type;
  using vector_type = typename base::vector_type;
  using matrix_type = typename base::matrix_type;

  Camera(int width, int height, value_type fov_x)
    : width_(width), height_(height),
      base({0,0,0}, {1,0,0}, {0,-1,0},{0,0,-1})
  {
    fov(fov_x);
  }

  void fov(value_type fov_x) {
    fov_x = std::min(std::max((value_type)0, fov_x), std::nextafter(blackhole::kPi<value_type>, (value_type)0));
    fov_ = fov_x;
    focus_len_ = width() / (2 * std::tan(fov_x / 2.0));
  }
  [[nodiscard]] value_type fov() const { return fov_; }

  [[nodiscard]] int width() const { return width_; }
  [[nodiscard]] int height() const { return height_; }

  [[nodiscard]] const vector_type& focus() const { return this->position(); }

  [[nodiscard]] vector_type PixelVector(int x, int y) const {
    return this->vector_x() * focus_len_
         - this->vector_y() * static_cast<value_type>(width() / 2.0 - x)
         - this->vector_z() * static_cast<value_type>(height() / 2.0 - y);
  }

  [[nodiscard]] vector_type PixelVector(int x, int y, const vector_type& focus_v) const {
    return focus_v
         - this->vector_y() * static_cast<value_type>(width() / 2.0 - x)
         - this->vector_z() * static_cast<value_type>(height() / 2.0 - y);
  }

  [[nodiscard]] vector_type focus_vector() const {
    return this->vector_x() * focus_len_;
  }

 private:
  int width_, height_;

  value_type focus_len_ = 1;
  value_type fov_ = pi / 2;
//  vector_type fv_ = vector_x();
};

} // namespace blackhole

#endif // BLACKHOLE_CAMERA_H_
