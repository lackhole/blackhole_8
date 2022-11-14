//
// Created by YongGyu Lee on 2022/11/12.
//

#ifndef BLACKHOLE_CAMERA_H_
#define BLACKHOLE_CAMERA_H_

#include <cmath>
#include <type_traits>

#include "opencv2/opencv.hpp"

#include "blackhole/matrix.h"
#include "blackhole/constants.h"

namespace blackhole {

class Camera {
 public:
  // TODO: Use custom matrices
  using value_type = double;
  using point_type = cv::Vec<value_type, 3>;
  using vector_type = cv::Vec<value_type, 3>;
  using matrix_type = cv::Matx<value_type, 3, 3>;

  Camera(int width, int height, double fov_x)
    : width_(width), height_(height),
      focus_len_(width / (2 * std::tan(fov_x / 2.0))),
      fov_(fov_x)
  {
    fv_ = xv_ * focus_len_;
  }

//  void Resize(int width, int height) {
//    width_ = width;
//    height_ = height;
//  }

//  void Resize(int width, int height, value_type fov) {
//
//  }

//  void fov(value_type fov_x) {
//    fov_ = fov_x;
//    focus_len_ = width() / (2 * std::tan(fov_x / 2.0));
//    focus_ = focus_ * (focus_len_ / focus_.dot(focus_));
//  }

  [[nodiscard]] value_type fov() const { return fov_; }

//  void ResizeX()

//  void width(int val) { }
  [[nodiscard]] int width() const { return width_; }

//  void height(int val) { }
  [[nodiscard]] int height() const { return height_; }

  [[nodiscard]] const point_type& position() const { return origin_; }
  [[nodiscard]] const vector_type& focus() const { return origin_; }

  [[nodiscard]] vector_type PixelVector(int x, int y) const {
    return fv_ - yv_ * static_cast<value_type>(width() / 2.0 - x) - zv_ * static_cast<value_type>(height() / 2.0 - y);
  }

  [[nodiscard]] const vector_type& VectorX() const { return xv_; }
  [[nodiscard]] const vector_type& VectorY() const { return yv_; }
  [[nodiscard]] const vector_type& VectorZ() const { return zv_; }

  static matrix_type RotationMatrixForAxis(const vector_type& v, double theta) {
    const auto u = cv::normalize(v);
    const auto ux = u[0];
    const auto uy = u[1];
    const auto uz = u[2];
    const auto c = std::cos(theta);
    const auto c2 = 1 - c;
    const auto s = std::sin(theta);
    const matrix_type r(
      c + ux*ux*c2, ux*uy*c2 - uz*s, ux*uz*c2 + uy*s,
      uy*ux*c2 + uz*s, c + uy*uy*c2, uy*uz*c2 - ux*s,
      uz*ux*c2 - uy*s, uz*uy*c2 + ux*s, c + uz*uz*c2
    );
    return r;
  }

  void RotateHorizontally(double theta) {
    const auto r = RotationMatrixForAxis(zv_, theta);
    xp_ = r * (xp_ - origin_) + origin_;
    yp_ = r * (yp_ - origin_) + origin_;
    xv_ = cv::normalize(xp_ - origin_);
    yv_ = cv::normalize(yp_ - origin_);
    fv_ = xv_ * focus_len_;
  }

  void RotateVertically(double phi) {
    const auto r = RotationMatrixForAxis(yv_, phi);
    xp_ = r * (xp_ - origin_) + origin_;
    zp_ = r * (zp_ - origin_) + origin_;
    xv_ = cv::normalize(xp_ - origin_);
    zv_ = cv::normalize(zp_ - origin_);
    fv_ = xv_ * focus_len_;
  }

  void RotateSelf(double a) {
    const auto r = RotationMatrixForAxis(xv_, a);
    yp_ = r * (yp_ - origin_) + origin_;
    zp_ = r * (zp_ - origin_) + origin_;
    yv_ = cv::normalize(yp_ - origin_);
    zv_ = cv::normalize(zp_ - origin_);
    fv_ = xv_ * focus_len_;
  }

  void Move(double x, double y, double z) {
    origin_[0] += x; origin_[1] += y; origin_[2] += z;
    xp_[0]     += x; xp_[1]     += y;     xp_[2] += z;
    yp_[0]     += x; yp_[1]     += y;     yp_[2] += z;
    zp_[0]     += x; zp_[1]     += y;     zp_[2] += z;
  }

  void MoveForward(double distance) {
    origin_ += xv_ * distance;
    xp_     += xv_ * distance;
    yp_     += xv_ * distance;
    zp_     += xv_ * distance;
  }

  void MoveSideways(double distance) {
    origin_ += yv_ * distance;
    xp_     += yv_ * distance;
    yp_     += yv_ * distance;
    zp_     += yv_ * distance;
  }

  void MoveVertical(double distance) {
    origin_ += zv_ * distance;
    xp_     += zv_ * distance;
    yp_     += zv_ * distance;
    zp_     += zv_ * distance;
  }

//  void LookAt(double x, double y, double z) {
//    const auto v = cv::normalize(vector_type(x, y, z));
//    xp_ = origin_ + v * xv_;
//    yp_ = origin_ + v * yv_;
//    zp_ = origin_ + v * zv_;
//  }

 private:
  int width_, height_;
  Vector<value_type, 2> pixel_size_{1, 1};

  point_type origin_ = {0, 0, 0};
  point_type xp_ = {1, 0, 0};
  point_type yp_ = {0, -1, 0};
  point_type zp_ = {0, 0, -1};

  vector_type xv_ = xp_ - origin_;
  vector_type yv_ = yp_ - origin_;
  vector_type zv_ = zp_ - origin_;

  value_type focus_len_;
  value_type fov_;
//  point_type fp_ = {0, 0, 0};
  vector_type fv_ = xv_;
};

} // namespace blackhole

#endif // BLACKHOLE_CAMERA_H_
