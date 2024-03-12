#pragma once

#include <vector>
#include <memory>
#include <string>

#include "huron/types.h"
#include "huron/control_interfaces/joint.h"
#include "joint_common.h"

namespace huron {
namespace multibody {
namespace internal {

template <typename T>
class ModelImplInterface {
 public:
  ModelImplInterface() = default;
  ModelImplInterface(const ModelImplInterface&) = delete;
  ModelImplInterface& operator=(const ModelImplInterface&) = delete;
  virtual ~ModelImplInterface() = default;

  virtual void BuildFromUrdf(const std::string& urdf_path,
                             JointType root_joint_type);

  virtual const std::vector<std::string>& GetJointNames() const;
  virtual std::weak_ptr<Joint<T>> GetJoint(const std::string& name) const;
  virtual std::weak_ptr<Joint<T>> GetJoint(size_t joint_index) const;

  virtual JointType GetJointType(size_t joint_index) const;
  virtual JointIndex GetJointIndex(const std::string& joint_name) const = 0;

  virtual std::unique_ptr<JointDescription<T>> GetJointDescription(
    JointIndex joint_index) const;
  virtual std::unique_ptr<JointDescription<T>> GetJointDescription(
    const std::string& joint_name) const;

  virtual huron::SE3<T>
  GetJointTransformInWorld(size_t joint_index) const;

  virtual FrameIndex GetFrameIndex(
    const std::string& frame_name) const;
  virtual const std::string& GetFrameName(FrameIndex frame_index) const;
  virtual FrameType GetFrameType(FrameIndex frame_index) const;
  virtual huron::SE3<T> GetFrameTransform(FrameIndex from_frame,
                                            FrameIndex to_frame) const;
  virtual huron::SE3<T> GetFrameTransformInWorld(FrameIndex frame) const;

  virtual huron::Vector3<T> EvalCenterOfMassPosition();
  virtual huron::Vector3<T> GetCenterOfMassPosition() const;

  virtual huron::VectorX<T> NeutralConfiguration() const;

  /**
   * @brief Get the generalized accelerations of the model.
   */
  virtual const huron::VectorX<T>& GetAccelerations() const;

  /**
   * @brief Get the joint torques.
   */
  virtual const huron::VectorX<T>& GetTorques() const;

  /**
   * @brief Get the mass matrix with the cached value.
   */
  virtual const huron::MatrixX<T>& GetMassMatrix() const;

  /**
   * @brief Get the Coriolis matrix with the cached value.
   */
  virtual const huron::MatrixX<T>& GetCoriolisMatrix() const;

  /**
   * @brief Get the nonlinear effects vector.
   */
  virtual const huron::VectorX<T>& GetNonlinearEffects() const;

  /**
   * @brief Get the gravity vector.
   */
  virtual const huron::VectorX<T>& GetGravity() const;

  /**
   * @brief Get the spatial momentum with respect to the specified frame.
   */
  virtual const huron::Vector6<T>& GetSpatialMomentum() const;

  /**
   * @brief Get the centroidal momentum.
   */
  virtual huron::Vector6<T> GetCentroidalMomentum() const;

  /**
   * @brief Get the centroidal momentum matrix with the cached value.
   */
  virtual const huron::Matrix6X<T>& GetCentroidalMatrix() const;

  virtual void ComputeAll(
    const Eigen::Ref<const huron::VectorX<T>>& q,
    const Eigen::Ref<const huron::VectorX<T>>& v);

  virtual void ForwardKinematics(
    const Eigen::Ref<const huron::VectorX<T>>& q);
  virtual void ForwardKinematics(
    const Eigen::Ref<const huron::VectorX<T>>& q,
    const Eigen::Ref<const huron::VectorX<T>>& v);
  virtual void ForwardKinematics(
    const Eigen::Ref<const huron::VectorX<T>>& q,
    const Eigen::Ref<const huron::VectorX<T>>& v,
    const Eigen::Ref<const huron::VectorX<T>>& a);

  virtual bool is_built() const;

  virtual size_t num_positions() const;
  virtual size_t num_velocities() const;
  virtual size_t num_joints() const;
  virtual size_t num_frames() const;
};

}  // namespace internal
}  // namespace multibody
}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::internal::ModelImplInterface)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::internal::ModelImplInterface)
