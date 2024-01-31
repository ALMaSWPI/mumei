#pragma once

#include <eigen3/Eigen/Core>

namespace huron {

template <typename T>
using Affine3 = Eigen::Transform<T, 3, Eigen::Affine>;

template <typename T>
using Vector3 = Eigen::Matrix<T, 3, 1>;

template <typename T>
using VectorX = Eigen::Matrix<T, Eigen::Dynamic, 1>;

template <typename T>
using MatrixX = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

template <typename T>
using Vector6 = Eigen::Matrix<T, 6, 1>;

template <typename T>
using Matrix6X = Eigen::Matrix<T, 6, Eigen::Dynamic>;

/// double aliases
using Affine3d = Affine3<double>;
using Vector3d = Vector3<double>;
using VectorXd = VectorX<double>;
using MatrixXd = MatrixX<double>;
using Vector6d = Vector6<double>;
using Matrix6Xd = Matrix6X<double>;

}  // namespace huron
