//
// Created by YongGyu Lee on 2022/11/14.
//

#include "blackhole/object/object.h"
#include "blackhole/utility.h"
#include "blackhole/constants.h"

using Object = blackhole::Object<double>;

template<typename T>
bool point_equal(const T& a, const T& b) {
  using namespace blackhole;
  return float_equal(a[0], b[0]) && float_equal(a[1], b[1]) && float_equal(a[2], b[2]);
}

int main() {

  using point = typename Object::point_type;
  using vector = typename Object::vector_type;

  {
    Object obj({0, 0, 0});
    obj.MoveTo(10, 10, 10);
    if (!point_equal(obj.position(), point(10, 10, 10))) return 1;

    obj.Move(11, 12, 13);
    if (!point_equal(obj.position(), point(21, 22, 23))) return 1;

    obj.MoveTo(0, 0, 0);
    obj.MoveX(12345);
    if (!point_equal(obj.position(), point(12345, 0, 0))) return 1;
    obj.MoveY(12345);
    if (!point_equal(obj.position(), point(12345, 12345, 0))) return 1;
    obj.MoveZ(12345);
    if (!point_equal(obj.position(), point(12345, 12345, 12345))) return 1;

    obj.RotateX(blackhole::pi / 2);
    if (!point_equal(obj.vector_x(), vector(1, 0, 0))) return 1;
    if (!point_equal(obj.vector_y(), vector(0, 0, 1))) return 1;
    if (!point_equal(obj.vector_z(), vector(0, -1, 0))) return 1;
    obj.RotateX(-blackhole::pi / 2);

    obj.RotateY(blackhole::pi / 2);
    if (!point_equal(obj.vector_x(), vector(0, 0, -1))) return 1;
    if (!point_equal(obj.vector_y(), vector(0, 1, 0))) return 1;
    if (!point_equal(obj.vector_z(), vector(1, 0, 0))) return 1;
    obj.RotateY(-blackhole::pi / 2);

    obj.RotateZ(blackhole::pi / 2);
    if (!point_equal(obj.vector_x(), vector(0, 1, 0))) return 1;
    if (!point_equal(obj.vector_y(), vector(-1, 0, 0))) return 1;
    if (!point_equal(obj.vector_z(), vector(0, 0, 1))) return 1;
    obj.RotateZ(-blackhole::pi / 2);
  }

  return 0;
}
