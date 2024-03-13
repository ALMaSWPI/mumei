#pragma once

#include "huron/types.h"
#include "huron/utils/template_instantiations.h"

namespace huron {

template <typename T>
class SE3 {
 public:
  SE3();
  SE3(const Eigen::Matrix4<T>& tf);
  SE3(const Eigen::Matrix3<T>& R, const Eigen::Vector3<T>& t);
  SE3(const SE3&);
  SE3& operator=(const SE3& other) {
    tf_ = other.tf_;
    return *this;
  }
  virtual ~SE3() = default;

  SE3 operator*(const SE3& other) const {
    SE3 ret = *this;
    ret.tf_ *= other.tf_;
    return ret;
  }

  huron::Vector3<T> operator*(const huron::Vector3<T>& v) const {
    huron::Vector4<T> v_homogeneous;
    v_homogeneous << v, 1;
    return (tf_ * v_homogeneous).template head<3>();
  }

  SE3& operator*=(const SE3& other) {
    tf_ *= other.tf_;
    return *this;
  }

  inline friend bool operator==(const SE3<double>& lhs, const SE3<double>& rhs);
  inline friend bool operator!=(const SE3<double>& lhs, const SE3<double>& rhs);

  Matrix4<T>& matrix() { return tf_; }
  const Matrix4<T>& matrix() const { return tf_; }

  Matrix3<T> rotation() const;
  Vector3<T> translation() const;

  void Rotate(const Matrix3<T>& R);
  void Prerotate(const Matrix3<T>& R);
  void Translate(const Vector3<T>& v);

  SE3<T> Inverse() const;

  huron::Matrix6<T> AdjointAction() const;

 private:
  Matrix4<T> tf_;
};

inline bool operator==(const SE3<double>& lhs, const SE3<double>& rhs) {
  return lhs.tf_ == rhs.tf_;
}
inline bool operator!=(const SE3<double>& lhs, const SE3<double>& rhs) {
  return !(lhs == rhs);
}

} // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::SE3)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::SE3)
