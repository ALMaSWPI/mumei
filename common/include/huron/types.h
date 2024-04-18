#pragma once

#include <eigen3/Eigen/Core>

namespace huron {

template <typename T>
using Affine3 = Eigen::Transform<T, 3, Eigen::Affine>;

template <typename T>
using Vector4 = Eigen::Matrix<T, 4, 1>;

template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
using Vector2 = Eigen::Matrix<T, 2, 1>;

template <typename T>
using VectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template <typename T>
using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T>
using Matrix3 = Eigen::Matrix<T, 3, 3>;

template <typename T>
using Matrix4 = Eigen::Matrix<T, 4, 4>;

template <typename T>
using Vector6 = Eigen::Matrix<T, 6, 1>;

template <typename T>
using Matrix6 = Eigen::Matrix<T, 6, 6>;

template <typename T>
using Matrix6X = Eigen::Matrix<T, 6, Eigen::Dynamic>;

/// double aliases
using Affine3d = Affine3<double>;
using Vector4d = Vector4<double>;
using Vector3d = Vector3<double>;
using Vector2d = Vector2<double>;
using VectorXd = VectorX<double>;
using MatrixXd = MatrixX<double>;
using Matrix3d = Matrix3<double>;
using Matrix4d = Matrix4<double>;
using Vector6d = Vector6<double>;
using Matrix6Xd = Matrix6X<double>;

}  // namespace huron

#if HURON_ENABLE_AUTODIFF==1
#if HURON_USE_CASADI==1

#include "huron/math/casadi.h"

// namespace Eigen {
//
// template <>
// struct NumTraits<casadi::SX> : NumTraits<double>
// {
//   using Real = casadi::SX;
//   using NonInteger = casadi::SX;
//   using Literal = casadi::SX;
//   using Nested = casadi::SX;
//   
//   enum {
//     // does not support complex Base types
//     IsComplex             = 0 ,
//     // does not support integer Base types
//     IsInteger             = 0 ,
//     // only support signed Base types
//     IsSigned              = 1 ,
//     // must initialize an AD<Base> object
//     RequireInitialization = 1 ,
//     // computational cost of the corresponding operations
//     ReadCost              = 1 ,
//     AddCost               = 2 ,
//     MulCost               = 2
//   };
// };

// }  // namespace Eigen

#endif  // HURON_USE_CASADI
#endif  // HURON_ENABLE_AUTODIFF
