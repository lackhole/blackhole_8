//
// Created by YongGyu Lee on 2022/11/15.
//

#ifndef BLACKHOLE_VECTOR_OBJECT_H_
#define BLACKHOLE_VECTOR_OBJECT_H_

#include <limits>

#include "blackhole/object/object.h"
#include "blackhole/object/material.h"
#include "blackhole/utility.h"

namespace blackhole {

template<typename T>
class Triangle : public DrawableObject<T> {
 public:
  using object = Object<T>;
  using value_type = typename object::value_type;
  using point_type = typename object::point_type;
  using vector_type = typename object::vector_type;
  using matrix_type = typename object::matrix_type;

  Triangle() : object({
    {1, 0, 0},
    {-1, 0, 0},
    {0, 0, 1.73}
  }) {}

  Triangle(
    value_type v11, value_type v12, value_type v13,
    value_type v21, value_type v22, value_type v23,
    value_type v31, value_type v32, value_type v33)
    : object({{v11, v12, v13}, {v21, v22, v23}, {v31, v32, v33}}) {}

  // https://en.wikipedia.org/wiki/M%C3%B6ller%E2%80%93Trumbore_intersection_algorithm
  bool Collide(const point_type& p1, const point_type& p2, point_type* intersection) const override {
    const value_type ep = std::numeric_limits<value_type>::epsilon() * 10;

    const auto ray_vector = p2 - p1;
    const auto& vertex0 = this->vertex()[1];
    const auto& vertex1 = this->vertex()[2];
    const auto& vertex2 = this->vertex()[3];
    const auto edge1 = vertex1 - vertex0;
    const auto edge2 = vertex2 - vertex0;
    const auto h = ray_vector.cross(edge2);
    const auto a = edge1.dot(h);

    if (-ep < a && a < ep)
      return false; // The ray is parallel to the triangle

    const auto f = static_cast<value_type>(1.0) / a;
    const auto s = p1 - vertex0;
    const auto u = f * s.dot(h);
    if (u < 0 || u > 1)
      return false;

    const auto q = s.cross(edge1);
    const auto v = f * ray_vector.dot(q);
    if (v < 0 || u + v > 1)
      return false;

    // At this stage we can compute t to find out where the intersection point is on the line.
    const auto t = f * edge2.dot(q);
    if (t <= ep)
      return false;

    *intersection = p1 + ray_vector * t;
    return true;
  }

  static double SignedTetraVolume(const point_type& a, const point_type& b, const point_type& c, const point_type& d) {
    return (b - a).cross(c - a).dot(d - a);
  }
};

template<typename T>
class Rectangle : public DrawableObject<T> {
 public:
  using object = DrawableObject<T>;
  using value_type = typename object::value_type;
  using point_type = typename object::point_type;
  using vector_type = typename object::vector_type;
  using matrix_type = typename object::matrix_type;

  Rectangle() : object({
    {1, 0, 1},
    {-1, 0, 1},
    {-1, 0, -1},
    {1, 0, -1}}) {}

  Rectangle(
    value_type v11, value_type v12, value_type v13,
    value_type v21, value_type v22, value_type v23,
    value_type v31, value_type v32, value_type v33,
    value_type v41, value_type v42, value_type v43)
  : object({{v11, v12, v13}, {v21, v22, v23}, {v31, v32, v33}, {v41, v42, v43}}) {}

  bool Collide(const point_type& aa, const point_type& bb, point_type *intersection) const override {
    const auto p0 = this->vertex()[1];
    const auto p1 = this->vertex()[2] - p0;
    const auto p3 = this->vertex()[4] - p0;
    const auto n = cv::normalize(p1.cross(p3));
    const auto Q1 = aa - p0;
    const auto Q2 = bb - p0;

    const auto q1 = n.dot(Q1);
    const auto q2 = n.dot(Q2);

    if (q1 * q2 >= 0) return false;

    const auto Q = (std::abs(q2) * Q1 + std::abs(q1) * Q2) / (std::abs(q1) + std::abs(q2));
    if (p1.dot(Q) > 0 && p1.dot(p1) > p1.dot(Q) &&
        p3.dot(Q) > 0 && p3.dot(p3) > p3.dot(Q)) {
      *intersection = Q + p0;
      return true;
    }
    return false;


//    const value_type ep = std::numeric_limits<value_type>::epsilon() * 10;
//    const auto d = cv::normalize(p2 - p1);
//    const auto& r0 = p1;
//
//    const auto& p0 = this->vertex()[1];
//    const auto s1 = this->vertex()[2] - p0;
//    const auto s2 = this->vertex()[4] - p0;
//    const auto n = cv::normalize(s1.cross(s2));
//
//    const auto s = d.dot(n);
//    if (-ep < s && s < ep)
//      return false;
//
//    const auto a = (p0 - r0).dot(n) / d.dot(n);
//    if (a < 0)
//      return false;
//
//    const auto p = r0 + a * d;
//    const auto r = p - p0;
//
//    if (r.dot(s1) < 0) return false;
//    if (r.dot(s1) > s1.dot(s1)) return false;
//    if (r.dot(s2) < 0) return false;
//    if (r.dot(s2) > s2.dot(s2)) return false;
//
//    *intersection = p;
//
//    return true;
  }

  cv::Vec3b color(double x, double y, double z) const override {
    return color({x, y, z});
  }

  cv::Vec3b color(const point_type& p) const {
    const auto s1 = this->vertex()[2] - this->vertex()[1];
    const auto s2 = this->vertex()[4] - this->vertex()[1];
    const auto v = p - this->vertex()[1];
    const auto r = std::sqrt(v.dot(v));
    auto theta = std::acos(s1.dot(v) / (std::sqrt(s1.dot(s1)) * r));

    theta = std::isnan(theta) ? 0 : theta;


    const int px_w = (int)((r * std::cos(theta) / std::sqrt(s1.dot(s1))) * this->texture_.cols);
    const int px_h = (int)((r * std::sin(theta) / std::sqrt(s2.dot(s2))) * this->texture_.rows);

    unsigned char* ptr = this->texture_.data + ((px_h * this->texture_.cols + px_w) * 3);

    return {ptr[0], ptr[1], ptr[2]};
  }

  void foo() {}
 private:

};

template<typename T>
class InfinitePlane : public DrawableObject<T> {
 public:
  using object = DrawableObject<T>;
  using value_type = typename object::value_type;
  using point_type = typename object::point_type;
  using vector_type = typename object::vector_type;
  using matrix_type = typename object::matrix_type;

  using object::vector_x;
  using object::vector_y;
  using object::vector_z;
  using object::position;

  InfinitePlane(const point_type& p)
    : object(p) {}

  InfinitePlane(const point_type& p, std::function<cv::Vec3b(value_type x, value_type y, value_type z)> pattern)
    : object(p), pattern_(pattern) {}

  InfinitePlane(const point_type& p, const cv::Vec3b& color)
    : object(p), pattern_([color = color]() { return color; })
  {}

  bool Collide(const point_type& p1, const point_type& p2, point_type *intersection) const override {
    const auto t1 = vector_z()[0] * (p1[0] - position()[0])
                  + vector_z()[1] * (p1[1] - position()[1])
                  + vector_z()[2] * (p1[2] - position()[2]);
    const auto t2 = vector_z()[0] * (p2[0] - position()[0])
                  + vector_z()[1] * (p2[1] - position()[1])
                  + vector_z()[2] * (p2[2] - position()[2]);

    if (t1 * t2 > 0)
      return false;

    const auto d1 = std::abs(t1);
    const auto d2 = std::abs(t2);
    *intersection = (d2 * p1 + d1 * p2) / (d1 + d2);
    return true;
  }

  [[nodiscard]] cv::Vec3b color(double x, double y, double z) const override {
    const auto b = (vector_x()[0] * y - vector_x()[1] * x) /
        (vector_x()[0] * vector_y()[1] - vector_x()[1] * vector_y()[0]);
    const auto a = (x - b * vector_y()[0]) / vector_x()[0];
    return pattern_(a, b, 0);
  }

 private:
  using object::object;
  using Material::SetTexture;
  std::function<cv::Vec3b(value_type x, value_type y, value_type z)> pattern_
    = [](auto...) -> cv::Vec3b { return {0,0,0}; };
};


} // namespace blackhole

#endif // BLACKHOLE_VECTOR_OBJECT_H_
