#pragma once

#include "huron/types.h"

namespace huron {

template <typename T>
inline T abs(const T& x) {
  return x < 0 ? -x : x;
}

#if HURON_USE_CASADI==1

template <>
inline casadi::SX abs<casadi::SX>(const casadi::SX& x) {
  return casadi::SX::abs(x);
}

#endif  // HURON_USE_CASADI==1
}  // namespace huron
