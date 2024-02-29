#include "huron/math/se3.h"

namespace huron {

template <typename T>
SE3<T>::SE3() : tf_(Matrix4<T>::Identity()) {}

template <typename T>
SE3<T>::SE3(const Eigen::Matrix4<T>& tf) : tf_(tf) {}

template <typename T>
SE3<T>::SE3(const Eigen::Matrix3<T>& R, const Eigen::Vector3<T>& t) {
  tf_.block(0, 0, 3, 3) = R;
  tf_.block(0, 3, 3, 1) = t;
  tf_.row(3) << 0, 0, 0, 1;
}

template <typename T>
SE3<T>::SE3(const SE3& other) : tf_(other.tf_) {}

template <typename T>
Matrix3<T> SE3<T>::rotation() const {
  return tf_.block(0, 0, 3, 3);
}

template <typename T>
Vector3<T> SE3<T>::translation() const {
  return tf_.block(0, 3, 3, 1);
}

template <typename T>
void SE3<T>::rotate(const Matrix3<T>& R) {
  tf_.block(0, 0, 3, 3) = tf_.block(0, 0, 3, 3);
}

template <typename T>
void SE3<T>::prerotate(const Matrix3<T>& R) {
  Matrix4<T> R_tf = Matrix4<T>::Identity();
  R_tf.block(0, 0, 3, 3) = R;
  tf_ = R_tf * tf_;
}

template <typename T>
void SE3<T>::translate(const Vector3<T>& v) {
  tf_.block(0, 3, 3, 1) += v;
}

template <typename T>
SE3<T> SE3<T>::inverse() const {
  SE3<T> ret;
  ret.tf_ = tf_.inverse();
  return ret;
}

} // namespace huron

HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::SE3)
HURON_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::SE3)
