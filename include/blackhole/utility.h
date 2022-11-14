//
// Created by YongGyu Lee on 2022/11/15.
//

#ifndef BLACKHOLE_UTILITY_H_
#define BLACKHOLE_UTILITY_H_

#include <cmath>
#include <type_traits>
#include <limits>

namespace blackhole {

template<typename T> struct type_identity { using type = T; };
template<typename T> using type_identity_t = typename type_identity<T>::type;

template<typename T>
inline auto epsilon() {
  static const auto e = std::cbrt(std::numeric_limits<T>::epsilon());
  return e;
}

template<typename T, typename U, std::enable_if_t<
  std::disjunction_v<std::is_floating_point<T>, std::is_floating_point<U>> , int> = 0>
bool float_equal(T x, U y) {
  return std::abs(x - y) <= epsilon<std::common_type_t<T, U>>();
}

} // namespace blackhole

#endif // BLACKHOLE_UTILITY_H_
