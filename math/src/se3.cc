#include "huron/math/se3.h"
#include "huron/math/utils.h"

namespace huron {

template <typename T>
SE3<T>::SE3() : tf_(Matrix4<T>::Identity()) {}

template <typename T>
SE3<T>::SE3(const Eigen::Matrix4<T>& tf) : tf_(tf) {}

template <typename T>
SE3<T>::SE3(const Eigen::Matrix3<T>& R, const Eigen::Vector3<T>& t) {
  tf_.topLeftCorner(3, 3) = R;
  tf_.topRightCorner(3, 1) = t;
  tf_.row(3) << 0, 0, 0, 1;
}

template <typename T>
SE3<T>::SE3(const SE3& other) : tf_(other.tf_) {}

template <typename T>
Matrix3<T> SE3<T>::rotation() const {
  return tf_.topLeftCorner(3, 3);
}

template <typename T>
Vector3<T> SE3<T>::translation() const {
  return tf_.topRightCorner(3, 1);
}

template <typename T>
void SE3<T>::Rotate(const Matrix3<T>& R) {
  tf_.template topLeftCorner<3, 3>() *= R;
}

template <typename T>
void SE3<T>::Prerotate(const Matrix3<T>& R) {
  Matrix4<T> R_tf = Matrix4<T>::Identity();
  R_tf.template topLeftCorner<3, 3>() = R;
  tf_ = R_tf * tf_;
}

template <typename T>
void SE3<T>::Translate(const Vector3<T>& v) {
  tf_.topRightCorner(3, 1) += tf_.topLeftCorner(3, 3) * v;
}

template <typename T>
SE3<T> SE3<T>::Inverse() const {
  Matrix4<T> ret_tf = Matrix4<T>::Identity();
  ret_tf.topLeftCorner(3, 3) = tf_.topLeftCorner(3, 3).transpose();
  ret_tf.topRightCorner(3, 1) = -tf_.topLeftCorner(3, 3).transpose() * tf_.topRightCorner(3, 1);
  return SE3<T>(ret_tf);
}

template <typename T>
Matrix6<T> SE3<T>::AdjointAction() const {
  Matrix6<T> ret = Matrix6<T>::Zero();
  ret.template topLeftCorner<3, 3>() = rotation();
  ret.template topRightCorner<3, 3>() = huron::skew(translation()) * rotation();
  ret.template bottomRightCorner<3, 3>() = rotation();
  return ret;
}

} // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::SE3)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::SE3)
