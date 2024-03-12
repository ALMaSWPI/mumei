#pragma once

#include <experimental/propagate_const>
#include <vector>
#include <string>
#include <memory>

#include "huron/types.h"
#include "huron/multibody/model_impl_interface.h"
#include "huron/multibody/joint_common.h"
#include "huron/exceptions/not_implemented_exception.h"

namespace huron {
namespace multibody {
namespace internal {

template <typename T>
class PinocchioModelImpl : public ModelImplInterface<T> {
 public:
  PinocchioModelImpl();
  PinocchioModelImpl(const PinocchioModelImpl&) = delete;
  PinocchioModelImpl& operator=(const PinocchioModelImpl&) = delete;
  ~PinocchioModelImpl() override;

  static bool IsAvailable() { return true; }

  void BuildFromUrdf(const std::string& urdf_path,
                     JointType root_joint_type) override;

  const std::vector<std::string>& GetJointNames() const override;
  std::weak_ptr<Joint<T>> GetJoint(const std::string& name) const override;
  std::weak_ptr<Joint<T>> GetJoint(size_t joint_index) const override;

  JointType GetJointType(size_t joint_index) const override;
  JointIndex GetJointIndex(const std::string& joint_name) const override;

  std::unique_ptr<JointDescription<T>> GetJointDescription(
    JointIndex joint_index) const override;
  std::unique_ptr<JointDescription<T>> GetJointDescription(
    const std::string& joint_name) const override;

  huron::SE3<T>
  GetJointTransformInWorld(size_t joint_index) const override;

  FrameIndex GetFrameIndex(const std::string& frame_name) const override;
  const std::string& GetFrameName(FrameIndex frame_index) const override;
  FrameType GetFrameType(FrameIndex frame_index) const override;
  huron::SE3<T> GetFrameTransform(FrameIndex from_frame,
                            FrameIndex to_frame) const override;
  huron::SE3<T>
  GetFrameTransformInWorld(FrameIndex frame) const override;

  huron::Vector3<T> EvalCenterOfMassPosition() override;
  huron::Vector3<T> GetCenterOfMassPosition() const override;

  huron::VectorX<T> NeutralConfiguration() const override;

  const huron::VectorX<T>& GetAccelerations() const override;
  const huron::VectorX<T>& GetTorques() const override;
  const huron::MatrixX<T>& GetMassMatrix() const override;
  const huron::MatrixX<T>& GetCoriolisMatrix() const override;
  const huron::VectorX<T>& GetNonlinearEffects() const override;
  const huron::VectorX<T>& GetGravity() const override;
  const huron::Vector6<T>& GetSpatialMomentum() const override;
  huron::Vector6<T> GetCentroidalMomentum() const override;
  const huron::Matrix6X<T>& GetCentroidalMatrix() const override;

  void ComputeAll(
    const Eigen::Ref<const huron::VectorX<T>>& q,
    const Eigen::Ref<const huron::VectorX<T>>& v) override;

  void ForwardKinematics(
    const Eigen::Ref<const huron::VectorX<T>>& q) override;
  void ForwardKinematics(
    const Eigen::Ref<const huron::VectorX<T>>& q,
    const Eigen::Ref<const huron::VectorX<T>>& v) override;
  void ForwardKinematics(
    const Eigen::Ref<const huron::VectorX<T>>& q,
    const Eigen::Ref<const huron::VectorX<T>>& v,
    const Eigen::Ref<const huron::VectorX<T>>& a) override;

  bool is_built() const override { return is_built_; }
  size_t num_positions() const override { return num_positions_; }
  size_t num_velocities() const override { return num_velocities_; }
  size_t num_joints() const override { return num_joints_; }
  size_t num_frames() const override { return num_frames_; }

 private:
  struct Impl;
  std::experimental::propagate_const<std::unique_ptr<Impl>> impl_;

  bool is_built_ = false;
  size_t num_positions_ = 0;
  size_t num_velocities_ = 0;
  size_t num_joints_ = 0;
  size_t num_frames_ = 0;
};

template <>
void PinocchioModelImpl<double>::BuildFromUrdf(const std::string& urdf_path,
                   JointType root_joint_type);

}  // namespace internal
}  // namespace multibody
}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::internal::PinocchioModelImpl)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::internal::PinocchioModelImpl)
