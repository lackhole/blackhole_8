//
// Created by YongGyu Lee on 2022/11/12.
//

#include "blackhole/matrix.h"
#include "opencv2/opencv.hpp"
#include <iostream>

struct foo {
  union {
    struct {int a,b,c,d;};
  };
};

int main() {
  blackhole::Matrix<int, 2, 2> m = {1,2,3,4};

  blackhole::Matrix<int, 2, 2> m2(m);

  auto m3 = m2 * 3;

  blackhole::Vector<int, 2> v1;
  blackhole::Vector<int, 2> v2(v1 * 2);

  cv::Vec3d v;
  v * 2;
  v.dot(v);
//  v.dot

//  v1 * 3.0;
//
//  v1.x;

  return 0;
}
