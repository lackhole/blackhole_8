//
// Created by YongGyu Lee on 2022/11/13.
//

#ifndef BLACKHOLE_CONSTANTS_H_
#define BLACKHOLE_CONSTANTS_H_

namespace blackhole {

template<typename T = double>
inline constexpr T kPi = static_cast<T>(3.141592653589793238462643383279);

template<typename T = double>
inline constexpr T kE = static_cast<T>(2.718281828459045235360287471352);

inline constexpr double pi = kPi<double>;
inline constexpr double e = kE<double>;

} // namespace blackhole

#endif // BLACKHOLE_CONSTANTS_H_
