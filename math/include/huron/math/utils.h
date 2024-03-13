#pragma once

#include "huron/types.h"

namespace huron {

template <typename T>
Matrix3<T> skew(const Vector3<T>& v) {
  Matrix3<T> ret;
  ret << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return ret;
}

}  // namespace huron
