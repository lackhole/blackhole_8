//
// Created by YongGyu Lee on 2022/11/14.
//

#ifndef BLACKHOLE_OBJECT_OBJECT_H_
#define BLACKHOLE_OBJECT_OBJECT_H_

#include <vector>

#include "opencv2/opencv.hpp"

#include "blackhole/matrix.h"
#include "blackhole/object/material.h"

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
  template<typename P = point_type, std::enable_if_t<!std::is_arithmetic_v<P>, int> = 0>
  explicit Object(const P& p) : vertex_({p}) {}

  template<typename P = point_type, std::enable_if_t<!std::is_arithmetic_v<P>, int> = 0>
  Object(std::initializer_list<P> il) {
    vertex_.insert(vertex_.end(), il);
  }

  template<typename Vec = vector_type, std::enable_if_t<!std::is_arithmetic_v<Vec>, int> = 0>
  Object(const Vec& vx, const Vec& vy, const Vec& vz)
    : vx_(cv::normalize(vx)), vy_(cv::normalize(vy)), vz_(cv::normalize(vz)) {}

  template<typename P = point_type, typename Vec = vector_type, std::enable_if_t<!std::is_arithmetic_v<Vec> && !std::is_arithmetic_v<P>, int> = 0>
  Object(const P& p, const Vec& vx, const Vec& vy, const Vec& vz)
    : vertex_({p}), vx_(cv::normalize(vx)), vy_(cv::normalize(vy)), vz_(cv::normalize(vz)) {}

  template<typename P = point_type, typename Vec = vector_type,
    std::enable_if_t<!std::is_arithmetic_v<Vec> && !std::is_arithmetic_v<P>, int> = 0>
  Object(const P& p, const Vec& vx, const Vec& vy, const Vec& vz, std::initializer_list<point_type> il)
    : vertex_({p}), vx_(cv::normalize(vx)), vy_(cv::normalize(vy)), vz_(cv::normalize(vz))
    {
      vertex_.insert(vertex_.end(), il);
    }

  void RotateZ(value_type rad) {
    const auto r = RotationMatrixForAxis<matrix_type>(vz_, rad);
    vx_ = cv::normalize(r * vx_);
    vy_ = cv::normalize(r * vy_);
    for (auto& v : vertex_)
      v = r * (v - position()) + position();
  }

  void RotateY(value_type rad) {
    const auto r = RotationMatrixForAxis<matrix_type>(vy_, rad);
    vx_ = cv::normalize(r * vx_);
    vz_ = cv::normalize(r * vz_);
    for (auto& v : vertex_)
      v = r * (v - position()) + position();
  }

  void RotateX(value_type rad) {
    const auto r = RotationMatrixForAxis<matrix_type>(vx_, rad);
    vy_ = cv::normalize(r * vy_);
    vz_ = cv::normalize(r * vz_);
    for (auto& v : vertex_)
      v = r * (v - position()) + position();
  }

  void MoveTo(const point_type& p) { vertex_[0] = p; }
  void MoveTo(value_type x, value_type y, value_type z) { vertex_[0] = point_type(x, y, z); }

  void Move(value_type dx, value_type dy, value_type dz) { MoveX(dx); MoveY(dy); MoveZ(dz); }
  void MoveX(value_type distance) { for (auto& v : vertex_) v += vx_ * distance; }
  void MoveY(value_type distance) { for (auto& v : vertex_) v += vy_ * distance; }
  void MoveZ(value_type distance) { for (auto& v : vertex_) v += vz_ * distance; }

  [[nodiscard]] const point_type& position() const { return vertex_[0]; }
  [[nodiscard]] const vector_type& vector_x() const { return vx_; }
  [[nodiscard]] const vector_type& vector_y() const { return vy_; }
  [[nodiscard]] const vector_type& vector_z() const { return vz_; }

  const std::vector<point_type>& vertex() const { return vertex_; }

  void name(std::string name) { name_ = std::move(name); }
  [[nodiscard]] const std::string& name() const { return name_; }

 protected:
  std::vector<point_type> vertex_ = {point_type(0, 0, 0)};

 private:
  vector_type vx_ = {1, 0, 0};
  vector_type vy_ = {0, 1, 0};
  vector_type vz_ = {0, 0, 1};
  std::string name_ = "Unnamed";
};

template<typename T>
class DrawableObject : public Object<T>, public Material {
 public:
  using object = Object<T>;
  using value_type = typename object::value_type;
  using point_type = typename object::point_type;
  using vector_type = typename object::vector_type;
  using matrix_type = typename object::matrix_type;

  using object::object;
  using Material::color;

  virtual bool Collide(const point_type& p1, const point_type& p2, point_type* intersection) const { return false; }
};

} // namespace blackhole

#endif // BLACKHOLE_OBJECT_OBJECT_H_
