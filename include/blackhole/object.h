//
// Created by YongGyu Lee on 2022/11/14.
//

#ifndef BLACKHOLE_OBJECT_H_
#define BLACKHOLE_OBJECT_H_

#include "opencv2/opencv.hpp"

#include "blackhole/matrix.h"

namespace blackhole {

template<typename Pos, typename Vec>
struct PointVector {
  Pos origin;
  Vec vector;
};

template<typename T>
class Object {
 public:
  using value_type = T;
  static_assert(std::is_floating_point_v<value_type>);

  using point_type = cv::Vec<value_type, 3>;
  using vector_type = cv::Vec<value_type, 3>;
  using matrix_type = cv::Matx<value_type, 3, 3>;

  Object() = default;
  explicit Object(const point_type& p) : position_(p) {}
  Object(const vector_type& vx, const vector_type& vy, const vector_type& vz)
    : vx_(cv::normalize(vx)), vy_(cv::normalize(vy)), vz_(cv::normalize(vz)) {}
  Object(const point_type& p, const vector_type& vx, const vector_type& vy, const vector_type& vz)
    : position_(p), vx_(cv::normalize(vx)), vy_(cv::normalize(vy)), vz_(cv::normalize(vz)) {}

  void RotateZ(value_type rad) {
    const auto r = RotationMatrixForAxis<matrix_type>(vz_, rad);
    vx_ = cv::normalize(r * vx_);
    vy_ = cv::normalize(r * vy_);
  }

  void RotateY(value_type rad) {
    const auto r = RotationMatrixForAxis<matrix_type>(vy_, rad);
    vx_ = cv::normalize(r * vx_);
    vz_ = cv::normalize(r * vz_);
  }

  void RotateX(value_type rad) {
    const auto r = RotationMatrixForAxis<matrix_type>(vx_, rad);
    vy_ = cv::normalize(r * vy_);
    vz_ = cv::normalize(r * vz_);
  }

  void MoveTo(const point_type& p) { position_ = p; }
  void MoveTo(value_type x, value_type y, value_type z) { position_ = point_type(x, y, z); }

  void Move(value_type dx, value_type dy, value_type dz) { MoveX(dx); MoveY(dy); MoveZ(dz); }
  void MoveX(value_type distance) { position_ += vx_ * distance; }
  void MoveY(value_type distance) { position_ += vy_ * distance; }
  void MoveZ(value_type distance) { position_ += vz_ * distance; }

  [[nodiscard]] const point_type& position() const { return position_; }
  [[nodiscard]] const vector_type& vector_x() const { return vx_; }
  [[nodiscard]] const vector_type& vector_y() const { return vy_; }
  [[nodiscard]] const vector_type& vector_z() const { return vz_; }

 private:
  point_type position_ = {0, 0, 0};
  vector_type vx_ = {1, 0, 0};
  vector_type vy_ = {0, 1, 0};
  vector_type vz_ = {0, 0, 1};
};

} // namespace blackhole

#endif // BLACKHOLE_OBJECT_H_
