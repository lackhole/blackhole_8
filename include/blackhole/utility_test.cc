//
// Created by YongGyu Lee on 2022/11/15.
//

#include "blackhole/utility.h"

int main() {

  if (!blackhole::float_equal(1.f, 1.0049f)) return 1;
  if (!blackhole::float_equal(1E+200 + 1E+180, 1E+200)) return 1;

  return 0;
}
