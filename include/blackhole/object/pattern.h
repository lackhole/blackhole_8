//
// Created by YongGyu Lee on 2022/11/16.
//

#ifndef BLACKHOLE_OBJECT_PATTERN_H_
#define BLACKHOLE_OBJECT_PATTERN_H_

#include <cmath>

#include "opencv2/opencv.hpp"

namespace blackhole {

template<typename T>
class ChessPattern2D {
 public:
  using value_type = T;

  ChessPattern2D() = default;
  explicit ChessPattern2D(int pattern_size) : pattern_size_(pattern_size) {}

  cv::Vec3b operator()(value_type x, value_type y, value_type /* z */) {
    if (x * y > 0) {
      const auto x2 = mod(x);
      const auto y2 = mod(y);

      if ((x2 <= pattern_size_ && y2 <= pattern_size_) || (pattern_size_ <= x2 && pattern_size_ <= y2)) {
        return {255, 255, 255};
      } else {
        return {0, 0, 0};
      }
    } else {
      const auto x2 = mod(x);
      const auto y2 = mod(y);
      if ((x2 <= pattern_size_ && y2 <= pattern_size_) || (pattern_size_ <= x2 && pattern_size_ <= y2)) {
        return {0, 0, 0};
      } else {
        return {255, 255, 255};
      }
    }
  }

 private:
  value_type mod(value_type a) {
    const auto b = std::abs(a);
    return b - ((int)(b) / (int)(pattern_size_ * 2)) * (pattern_size_ * 2);
  }

  value_type pattern_size_ = 1;
};

} // namespace blackhole

#endif // BLACKHOLE_OBJECT_PATTERN_H_
