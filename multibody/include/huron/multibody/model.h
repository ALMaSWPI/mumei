#pragma once

#include <utility>
#include <memory>
#include <vector>
#include <string>
#include <unordered_map>

#include "huron/types.h"
#include "huron/utils/template_instantiations.h"
#include "huron/multibody/model_impl_types.h"
#include "huron/multibody/model_impl_interface.h"
#include "huron/multibody/frame.h"

namespace huron {
namespace multibody {

template <typename T>
class Model : public std::enable_shared_from_this<Model<T>> {
  using ModelImplInterface = internal::ModelImplInterface<T>;

 public:
  Model();
  Model(const Model&) = delete;
  Model& operator=(const Model&) = delete;
  ~Model() = default;

  /**
   * Adds a model implementation to the model.
   *
   * @param type The type of the model implementation.
   * @param set_as_default If true, the model implementation will be set as
   *                       the default model implementation.
   * @throws std::runtime_error if the model implementation is not available.
   */
  void AddModelImpl(ModelImplType type, bool set_as_default = false);

  /**
   * Adds a joint to the model.
   *
   * @param index The index of the joint.
   * @param args Arguments to be passed to the constructor of the joint.
   */
  template<typename ...Args>
  void AddJoint(JointIndex index,
                       Args&&... args) {
    assert(!is_constructed_);
    assert(!is_finalized_);
    if (joints_[index] != nullptr) {
      // TODO(dtbpkmte): provide index information in the error message.
      throw std::runtime_error("Joint already exists at this index.");
    }
    joints_[index] = std::make_shared<Joint<T>>(std::forward<Args>(args)...);
  }

  Joint<T>* const GetJoint(JointIndex index);
  Joint<T>* const GetJoint(const std::string& name);

  void SetJointStateProvider(JointIndex index,
                             std::shared_ptr<StateProvider<T>> state_provider);

  JointIndex GetJointIndex(const std::string& joint_name) const;
  /**
   * Adds a frame to the model.
   * Currently supported formats for external users:
   * - AddFrame<LogicalFrame>(name, parent_frame, transform_function)
   *
   * @note As of now, frames can only be added after the model is built from
   *      URDF. This will be changed in the future.
   *
   * @param index The index of the frame.
   * @param args Arguments to be passed to the constructor of the frame.
   */
  // template<typename FrameImpl, typename ...Args>
  // std::weak_ptr<const Frame>
  // AddFrame(const std::string& name, Args&&... args);

  template<typename FrameImpl, typename ...Args>
  std::weak_ptr<const Frame<T>> AddFrame(const std::string& name, Args&&... args) {
    static_assert(
      std::is_base_of_v<Frame<T>, FrameImpl>,
      "Invalid frame type.");
    static_assert(
      std::is_base_of_v<enable_protected_make_shared<FrameImpl>, FrameImpl>,
      "Frame-derived class must also derive from "
      "enable_protected_make_shared.");
    assert(is_constructed_);
    assert(!is_finalized_);

    DoAddFrame<FrameImpl>(name, true, std::forward<Args>(args)...);
    return frames_.back();
  }

  std::weak_ptr<const Frame<T>> GetFrame(FrameIndex index) const;
  std::weak_ptr<const Frame<T>> GetFrame(const std::string& name) const;

  void BuildFromUrdf(const std::string& urdf_path,
                     JointType root_joint_type = JointType::kFixed);

  /**
   * Performs final configuration and checks the validity of the model:
   * - Checks if all joints are added to the model.
   * - Adjusts the joint state indices in the states vector.
   * This method also sets the initial state [q, v] of the model.
   * @throws std::runtime_error if the model is not valid.
   */
  void Finalize(const huron::VectorX<T>& initial_state);
  void Finalize();

  /**
   * Updates the joint states [q, v] of the model.
   */
  void UpdateJointStates();

  void SetDefaultModelImpl(size_t index) {
    default_impl_index_ = index;
  }
  size_t GetDefaultModelImpl() const {
    return default_impl_index_;
  }


  // Kinematics and Dynamics wrapper functions
  huron::SE3<T> GetJointTransformInWorld(size_t joint_index) const;

  FrameIndex GetFrameIndex(const std::string& frame_name) const;
  const std::string& GetFrameName(FrameIndex frame_index) const;
  huron::SE3<T> GetFrameTransform(FrameIndex from_frame,
                                    FrameIndex to_frame) const;
  huron::SE3<T> GetFrameTransformInWorld(FrameIndex frame) const;

  huron::VectorX<T> NeutralConfiguration() const;

  huron::Vector3<T> EvalCenterOfMassPosition();
  huron::Vector3<T> GetCenterOfMassPosition() const;

  const Eigen::VectorBlock<const huron::VectorX<T>> GetPositions() const;

  const Eigen::VectorBlock<const huron::VectorX<T>> GetVelocities() const;

  /**
   * @brief Get the generalized accelerations of the model.
   */
  const huron::VectorX<T>& GetAccelerations() const;

  /**
   * @brief Get the joint torques.
   */
  const huron::VectorX<T>& GetTorques() const;

  /**
   * @brief Get the mass matrix with the cached value.
   */
  const huron::MatrixX<T>& GetMassMatrix() const;

  /**
   * @brief Get the Coriolis matrix with the cached value.
   */
  const huron::MatrixX<T>& GetCoriolisMatrix() const;

  /**
   * @brief Get the nonlinear effects vector.
   */
  const huron::VectorX<T>& GetNonlinearEffects() const;

  /**
   * @brief Get the gravity vector.
   */
  const huron::VectorX<T>& GetGravity() const;

  /**
   * @brief Get the spatial momentum with respect to the specified frame.
   */
  const huron::Vector6<T>& GetSpatialMomentum() const;

  /**
   * @brief Get the centroidal momentum.
   */
  huron::Vector6<T> GetCentroidalMomentum() const;

  /**
   * @brief Get the centroidal momentum matrix with the cached value.
   */
  const huron::Matrix6X<T>& GetCentroidalMatrix() const;

  void ComputeAll();

  void ForwardKinematics();

  bool is_finalized() const {
    return is_finalized_;
  }

  size_t num_positions() const {
    return num_positions_;
  }

  size_t num_velocities() const {
    return num_velocities_;
  }

  size_t num_joints() const {
    return joints_.size();
  }

  size_t num_frames() const {
    return frames_.size();
  }

 protected:
  /**
   * Returns the model implementation at the given index.
   * This function is provided for testing subclasses only.
   */
  ModelImplInterface const * GetModelImpl(size_t index) const;

  // template<typename FrameImpl, typename ...Args>
  // void DoAddFrame(const std::string& name, bool is_user_defined,
  //                 Args&&... args);
  template<typename FrameImpl, typename ...Args>
  void DoAddFrame(const std::string& name, bool is_user_defined,
                          Args&&... args) {
    // Check if the frame name already exists
    if (frame_name_to_index_.find(name) != frame_name_to_index_.end()) {
      throw std::runtime_error("Frame name already exists.");
    }
    frames_.push_back(FrameImpl::make_shared(
                        frames_.size(),  // frame index
                        name,  // frame name
                        is_user_defined,
                        this->weak_from_this(),  // model
                        std::forward<Args>(args)...));
    frame_name_to_index_[name] = frames_.size() - 1;
  }

  void DoAddFrameFromModelDescription(FrameIndex idx,
                                      const std::string& name,
                                      FrameType type);

  size_t default_impl_index_ = 0;
  std::vector<std::unique_ptr<ModelImplInterface>> impls_;
  std::vector<std::shared_ptr<Joint<T>>> joints_;
  /// \brief The joint states [q, v].
  huron::VectorX<T> states_;
  size_t num_positions_ = 0;
  size_t num_velocities_ = 0;

  /// \brief All frames, including those defined by the model description file
  /// and user-defined ones.
  std::vector<std::shared_ptr<Frame<T>>> frames_;
  std::unordered_map<std::string, FrameIndex> frame_name_to_index_;

  bool is_constructed_ = false;
  bool is_finalized_ = false;
};

}  // namespace multibody
}  // namespace huron

HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
    class huron::multibody::Model)
HURON_DECLARE_CLASS_TEMPLATE_INSTANTIATIONS_ON_AD_SCALARS(
    class huron::multibody::Model)
