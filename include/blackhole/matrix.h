//
// Created by YongGyu Lee on 2022/11/12.
//

#ifndef BLACKHOLE_MATRIX_H_
#define BLACKHOLE_MATRIX_H_

#include <cstddef>
#include <cmath>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

namespace blackhole {

template<typename T, size_t N>
struct MatrixElement {
  T elem[N];
};

template<typename T>
struct MatrixElement<T, 1> {
  union {
    T x;
    T elem[1];
  };
};

template<typename T>
struct MatrixElement<T, 2> {
  union {
    struct { T x, y; };
    T elem[2];
  };
};

template<typename T>
struct MatrixElement<T, 3> {
  union {
    struct { T x, y, z; };
    T elem[3];
  };
};

template<typename T>
struct MatrixElement<T, 4> {
  union {
    T elem[4];
    struct { T x, y, z, w; };
    struct { T v00, v01, v10, v11; };
  };
};

template<typename T, size_t N, bool v = (N > 100'000)>
struct MatrixElementBase {
  using value_type = T;

  constexpr value_type& operator[](size_t i) { return elem[i]; }
  constexpr const value_type& operator[](size_t i) const { return elem[i]; }

  MatrixElement<T, N> elem;
};

//template<typename T, size_t N>
//struct MatrixElementBase<T, N, true> {
// private:
//  template<size_t L> struct construct_va_arg_t {};
//
//  template<size_t L, typename ...U, size_t ...I>
//  constexpr MatrixElementBase(construct_va_arg_t<L>, std::index_sequence<I...>, U&&... args) {
//    auto dummy = {
//      (elem[I] = std::forward<U>(args), 0)...
//    };
//  }
//
// public:
//  using value_type = T;
//
//  constexpr MatrixElementBase() = default;
//
//  template<typename ...U, size_t L = sizeof...(U), std::enable_if_t<
//    std::conjunction_v<
//      std::is_constructible<value_type, U&&>...,
//      std::bool_constant<(L <= N)>
//    >,
//  int> = 0>
//  constexpr MatrixElementBase(U&&... args)
//      : MatrixElementBase(construct_va_arg_t<L>{}, std::make_index_sequence<L>(), std::forward<U>(args)...) {}
//
//  template<typename U>
//  constexpr MatrixElementBase(std::initializer_list<U> il) {
//    std::copy(il.begin(), il.end(), elem.begin());
//  }
//
//  constexpr value_type& operator[](size_t i) { return elem[i]; }
//  constexpr const value_type& operator[](size_t i) const { return elem[i]; }
//
// private:
//  std::vector<T> elem = std::vector<T>(N);
//};

template<typename T, size_t m, size_t n>
class Matrix : public MatrixElement<T, m * n> {
 public:
  enum {
    rows = m,
    cols = n,
    size = m * n,
  };

  using base = MatrixElement<T, m * n>;
  using base::elem;
  using value_type = T;

  constexpr value_type& operator[](size_t i) { return elem[i]; }
  constexpr const value_type& operator[](size_t i) const { return elem[i]; }

  constexpr value_type& operator()(size_t i, size_t j) { return elem[cols * i + j]; }
  constexpr const value_type& operator()(size_t i, size_t j) const { return elem[cols * i + j]; }

  double dot(const Matrix<T, m, n>& rhs) const {
    value_type s = 0;
    for( int i = 0; i < size; i++ ) s += elem[i] * rhs[i];
    return s;
  }

  double det() const {
    double net = 0;
    for (int i = 0; i < size; ++i) {
      net += elem[i];
    }
    return std::sqrt(net);
  }

  Matrix& resize(double ratio) {
    for (int i = 0; i < size; ++i) {
      elem[i] *= ratio;
    }
  }
};

template<typename T, size_t m>
class Vector : public Matrix<T, m, 1> {
 public:
  using base = Matrix<T, m, 1>;
//  using base::elem;

};

template<typename T>
using Point = Vector<T, 3>;



template<typename T, size_t m, size_t n, typename U>
Matrix<T, m, n> operator*(const Matrix<T, m, n>& lhs, const U& rhs) {
  Matrix<T, m, n> result;
  for (int i = 0; i < m * n; ++i) {
    result[i] = lhs[i] * rhs;
  }
  return result;
}

template<typename T, size_t m, size_t n, size_t l>
Matrix<T, m, n> operator*(const Matrix<T, m, l>& lhs, const Matrix<T, l, n>& rhs) {
  Matrix<T, m, n> result;
  for (size_t i = 0; i < m; ++i) {
    for (size_t j = 0; j < n; ++j) {
      for (size_t k = 0; k < l; ++k) {
        result(i, j) += lhs(i, k) * rhs(k, j);
      }
    }
  }
  return result;
}

template<typename T, size_t m, size_t n, typename U>
Matrix<T, m, n>& operator*=(Matrix<T, m, n>& v, const U& x) {
  for (int i = 0; i < m * n; ++i) {
    v[i] *= x;
  }
  return v;
}

template<typename T, size_t m, typename U>
Vector<T, m> operator*(const Vector<T, m>& v, const U& x) {
  Vector<T, m> result;
  for (int i = 0; i < m; ++i) {
    result[i] = v[i] * x;
  }
  return result;
}

template<typename T, size_t m, typename U>
Vector<T, m>& operator*=(Vector<T, m>& v, const U& x) {
  for (int i = 0; i < m; ++i) {
    v[i] *= x;
  }
  return v;
}

template<typename T, size_t m>
Vector<T, m> operator*(const T& x, const Vector<T, m>& v) {
  Vector<T, m> result;
  for (int i = 0; i < m; ++i) {
    result[i] = v[i] * x;
  }
  return result;
}

template<typename T, size_t m, typename U>
Vector<T, m> operator/(const Vector<T, m>& v, const U& x) {
  Vector<T, m> result;
  for (int i = 0; i < m; ++i) {
    result[i] = v[i] / x;
  }
  return result;
}

template<typename T, size_t m, typename U>
Vector<T, m>& operator/=(Vector<T, m>& v, const U& x) {
  for (int i = 0; i < m; ++i) {
    v[i] /= x;
  }
  return v;
}

template<typename T, size_t m, typename U>
Vector<T, m> operator/(const U& x, const Vector<T, m>& v) {
  Vector<T, m> result;
  for (int i = 0; i < m; ++i) {
    result[i] = v[i] / x;
  }
  return result;
}

template<typename T, size_t m, typename U>
Vector<T, m> operator+(const Vector<T, m>& v, const U& x) {
  Vector<T, m> result;
  for (int i = 0; i < m; ++i) {
    result[i] = v[i] + x;
  }
  return result;
}

template<typename T, size_t m, typename U>
Vector<T, m>& operator+=(Vector<T, m>& v, const U& x) {
  for (int i = 0; i < m; ++i) {
    v[i] += x;
  }
  return v;
}

template<typename T, size_t m, typename U>
Vector<T, m> operator+(const U& x, const Vector<T, m>& v) {
  Vector<T, m> result;
  for (int i = 0; i < m; ++i) {
    result[i] = v[i] + x;
  }
  return result;
}

template<typename T, size_t m, typename U>
Vector<T, m> operator-(const Vector<T, m>& v, const U& x) {
  Vector<T, m> result;
  for (int i = 0; i < m; ++i) {
    result[i] = v[i] - x;
  }
  return result;
}

template<typename T, size_t m, typename U>
Vector<T, m>& operator-=(Vector<T, m>& v, const U& x) {
  for (int i = 0; i < m; ++i) {
    v[i] -= x;
  }
  return v;
}

template<typename T, size_t m, typename U>
Vector<T, m> operator-(const Vector<T, m>& v) {
  Vector<T, m> result;
  for (int i = 0; i < m; ++i) {
    result[i] = -v[i];
  }
  return result;
}

template<typename T, size_t m>
Vector<T, m> operator+(const Vector<T, m>& lhs, const Vector<T, m>& rhs) {
  Vector<T, m> result;
  for (int i = 0; i < m; ++i) {
    result[i] = lhs[i] + rhs[i];
  }
  return result;
}

template<typename T, size_t m>
Vector<T, m>& operator+=(Vector<T, m>& lhs, const Vector<T, m>& rhs) {
  for (int i = 0; i < m; ++i) {
    lhs[i] += rhs[i];
  }
  return lhs;
}

template<typename T, size_t m>
Vector<T, m> operator-(const Vector<T, m>& lhs, const Vector<T, m>& rhs) {
  Vector<T, m> result;
  for (int i = 0; i < m; ++i) {
    result[i] = lhs[i] - rhs[i];
  }
  return result;
}

template<typename T, size_t m>
Vector<T, m>& operator-=(Vector<T, m>& lhs, const Vector<T, m>& rhs) {
  for (int i = 0; i < m; ++i) {
    lhs[i] -= rhs[i];
  }
  return lhs;
}

//template<typename T, size_t m, typename U>
//Vector<T, m> operator*(const Vector<T, m> lhs, const Vector<T, m>& rhs) {
//  Vector<T, m> result;
//  for (int i = 0; i < m; ++i) {
//    result[i] = lhs[i] - rhs;
//  }
//  return result;
//}

//template<typename T>
//cla

template<typename T, size_t m>
Vector<T, m> normalize(const Vector<T, m>& v) {
  auto s = v.dot(v);
  return v * (1.0 / s);
}

template<typename Matrix, typename Vec>
static Matrix RotationMatrixForAxis(const Vec& v, double theta) {
  const auto u = cv::normalize(v);
  const auto ux = u[0];
  const auto uy = u[1];
  const auto uz = u[2];
  const auto c = std::cos(theta);
  const auto c2 = 1 - c;
  const auto s = std::sin(theta);
  return Matrix(
    c + ux*ux*c2, ux*uy*c2 - uz*s, ux*uz*c2 + uy*s,
    uy*ux*c2 + uz*s, c + uy*uy*c2, uy*uz*c2 - ux*s,
    uz*ux*c2 - uy*s, uz*uy*c2 + ux*s, c + uz*uz*c2
  );
}

} // namespace blackhole

#endif // BLACKHOLE_MATRIX_H_
