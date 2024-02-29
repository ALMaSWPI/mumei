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

  SE3& operator*=(const SE3& other) {
    tf_ *= other.tf_;
    return *this;
  }

  Matrix4<T>& matrix() { return tf_; }
  const Matrix4<T>& matrix() const { return tf_; }

  Matrix3<T> rotation() const;
  Vector3<T> translation() const;

  void rotate(const Matrix3<T>& R);
  void prerotate(const Matrix3<T>& R);
  void translate(const Vector3<T>& v);

  SE3<T> inverse() const;

 private:
  Matrix4<T> tf_;
};

} // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::SE3)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::SE3)
