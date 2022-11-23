//
// Created by YongGyu Lee on 2022/11/16.
//

#ifndef BLACKHOLE_MATH_H_
#define BLACKHOLE_MATH_H_

#include <cmath>
#include <cstddef>
#include <type_traits>

#include "opencv2/opencv.hpp"

namespace blackhole {
namespace math {

template<typename T, size_t n>
auto size(const cv::Vec<T, n>& v) {
  return std::sqrt(v.dot(v));
}

template<template<typename, size_t> class V, typename T, size_t n>
T abs(const V<T, n>& v) {
  return size(v);
}

template<typename T, std::enable_if_t<std::is_arithmetic_v<T>, int> = 0>
T abs(T x) {
  return std::abs(x);
}

template<typename V>
auto angle(const V& v1, const V& v2) {
  return std::acos(v1.dot(v2) / (size(v1) * size(v2)));
}

template<typename P, typename U, std::enable_if_t<std::is_floating_point_v<U>, int> = 0>
P divide(const P& from, const P& to, U ratio) {
  return from + (to - from) * ratio;
}

//template<typename Point>
//double distanceBetweenLineSegmentAndPoint(const Point& l1, const Point& l2, const Point& point) {
//  if (l1 == l2)
//    return size(l1 - point);
//
//  auto p_close = l1;
//  auto p_far = l1;
//  if(size(l1 - point) > size(l2 - point))
//    p_close = l2;
//  else
//    p_far = l2;
//
//  auto u = p_close - p_far;
//  const auto lineSegmentLength = u.norm(u);
//
//  Point vec_l1 = p_far - point;
//  double theta = u.AngleWith(vec_l1);
//  double r = vec_l1.Size3();
//  double d = sin(theta) * r;
//  Vector4 vec_l1_d = (p_close - p_far).Normalize3() * r*cos(theta);
//  if((vec_l1_d.Size3() > u.Size3())){
//    return -1;
//  }
//  return d;
//}

template<typename T>
class FixedStepGenerator {
 public:
  using value_type = T;
  FixedStepGenerator(value_type first, value_type last, size_t step_count)
    : first_(first), last_(last), step_count_(step_count) {}

  value_type operator()(value_type prev) {

  }

 private:
  value_type first_;
  value_type last_;
  size_t step_count_;
};

template<typename T>
class DynamicStepGenerator {
 public:

 private:

};

template<typename AreaCalculator, typename StepGenerator>
class Integraph {
 public:

 private:
  AreaCalculator area_calculator_;
  StepGenerator step_generator_;
};

} // namespace math
} // namespace blackhole

#endif // BLACKHOLE_MATH_H_
