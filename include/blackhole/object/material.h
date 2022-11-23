//
// Created by YongGyu Lee on 2022/11/15.
//

#ifndef BLACKHOLE_MATERIAL_H_
#define BLACKHOLE_MATERIAL_H_

#include <exception>
#include <functional>
#include <iostream>
#include <utility>
#include <variant>

#include "opencv2/opencv.hpp"

namespace blackhole {

template<typename T>
class Material {
 public:
  using value_type = T;
  using function_type = std::function<cv::Scalar(value_type x, value_type y, value_type z)>;
  using texture_type = cv::Mat;
  using color_type = cv::Scalar;

  Material() = default;
//  explicit Material(cv::Mat texture) : material_(texture) {}

  void SetTexture(texture_type texture) {
    if (texture.empty()) {
      std::cerr << "Empty texture!\n";
      std::terminate();
    }
    texture_ = std::move(texture);
  }

  void SetColor(const cv::Vec3b& color) {
    texture_ = cv::Mat(2, 2, CV_8UC3, color);
  }

//  void SetPattern(function_type pattern) {
//    material_.emplace<function_type>(std::move(pattern));
//  }
//
//  cv::Scalar color(double x, double y, double z) {
//    return std::visit([=](auto&& arg) -> color_type {
//      using T = std::decay_t<decltype(arg)>;
//      if constexpr (std::is_same_v<T, texture_type>) return {};
//      else if constexpr (std::is_same_v<T, color_type>) return arg;
//      else return arg(x, y, z);
//    }, material_);
//  }

  virtual cv::Vec3b color(value_type x, value_type y, value_type z) const { return {0, 0, 0}; }

  template<typename P>
  cv::Vec3b color(const P& p) const { return color(p[0], p[1], p[2]); }

// private:
  texture_type texture_;
//  std::variant<function_type, texture_type, color_type> material_;
};

} // namespace blackhole

#endif // BLACKHOLE_MATERIAL_H_
