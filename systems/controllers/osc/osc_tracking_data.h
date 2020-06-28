#pragma once

#include <string>
#include <vector>
#include <Eigen/Dense>
#include <drake/multibody/plant/multibody_plant.h>
#include <drake/common/trajectories/trajectory.h>

#include "systems/framework/output_vector.h"
#include "systems/controllers/osc/osc_user_defined_pos.h"


namespace dairlib {
namespace systems {
namespace controllers {

/// OscTrackingData is a virtual class

/// Input of the constructor:
/// - dimension of the output/trajectory
/// - gains of PD controller
/// - cost weight
/// - a flag indicating the trajectory is a constant
/// - a flag indicating the trajectory has exponential term (that is, the traj
///   is of ExponentialPlusPiecewisePolynomial class)

/// In OSC, the position and the velocity of the robot are given. The goal is to
/// find the optimal
///   1. the acceleration of the robot
///   2. the input of the robot and
///   3. the contact force
/// with which the controller can track the trajectory the best.
/// In this class, we let
///   - y_ (ydot_) be the position (velocity) output of the robot
///   - y_des_ (ydot_des_) be the desired value of the position (velocity) output
///   - error_y_ (error_ydot_) be the error between the desired and the real
///     value of the position (velocity) output
/// Additionally, we let yddot_des_ be the desired acceleration for the output
/// (the second time derivative of the desired trajectory). In the case of
/// tracking a rotation output, the desired trajectory is represented in
/// quaternion (4 dimensional). Therefore, we need to convert yddot_des_ (the
/// second time derivatives of quaternion) into yddot_des_converted_ which is
/// the rotational acceleration.

/// In OSC, the cost function used for tracking a trajectory is
///   (J_*dv + JdotV - yddot_command)^T * W_ * (J_*dv + JdotV - yddot_command) / 2,
///   where
///   dv is the decision variable of QP, and
///   yddot_command = K_p_ * error_y_ + K_d_ * error_ydot_ + yddot_des_ with
///     error_y_ = y_des_ - y (except the rotational case), and
///     error_ydot_ = ydot_des_ - J_ * v
/// We ignore the constant term in cost function, since it doesn't affect
/// solution.

/// After the QP solver gives the solution to dv, then we can plug this
/// solution back to J_*dv + JdotV. Let the solution be dv_sol. Then the
/// acceleration of the trajectory generated by the QP is
///   yddot_command_sol_ = J_* dv_sol + JdotV

/// error_y_, error_ydot_, yddot_des_, JdotV and J_ are implemented in the
/// derived class.

/// Users can implement their own derived classes if the current
/// implementation here is not comprehensive enough.

class OscTrackingData {
 public:
  OscTrackingData(
      const std::string& name, int n_r, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr);

  // Update() updates the caches. It does the following things in order:
  //  - update track_at_current_state_
  //  - update desired output
  //  - update feedback output (Calling virtual methods)
  //  - update command output (desired output with pd control)
  // Inputs/Arguments:
  //  - `x_w_spr`, state of the robot (with spring)
  //  - `context_w_spr`, plant context of the robot (without spring)
  //  - `x_wo_spr`, state of the robot (with spring)
  //  - `context_wo_spr`, plant context of the robot (without spring)
  //  - `traj`, desired trajectory
  //  - `t`, current time
  //  - `finite_state_machine_state`, current finite state machine state
  bool Update(const Eigen::VectorXd& x_w_spr,
              drake::systems::Context<double>& context_w_spr,
              const Eigen::VectorXd& x_wo_spr,
              drake::systems::Context<double>& context_wo_spr,
              const drake::trajectories::Trajectory<double>& traj, double t,
              int finite_state_machine_state);

  // Getters for debugging
  Eigen::VectorXd GetY() const { return y_; }
  Eigen::VectorXd GetYDes() const { return y_des_; }
  Eigen::VectorXd GetErrorY() const { return error_y_; }
  Eigen::VectorXd GetYdot() const { return ydot_; }
  Eigen::VectorXd GetYdotDes() const { return ydot_des_; }
  Eigen::VectorXd GetErrorYdot() const { return error_ydot_; }
  Eigen::VectorXd GetYddotDes() const { return yddot_des_; }
  Eigen::VectorXd GetYddotDesConverted() const { return yddot_des_converted_; }
  Eigen::VectorXd GetYddotCommandSol() const { return yddot_command_sol_; }

  // Getters used by osc block
  Eigen::VectorXd GetOutput() const { return y_; }
  Eigen::MatrixXd GetJ() const { return J_; }
  Eigen::VectorXd GetJdotTimesV() const { return JdotV_; }
  Eigen::VectorXd GetYddotCommand() const { return yddot_command_; }
  Eigen::MatrixXd GetWeight() const { return W_; }

  // Getters
  std::string GetName() const { return name_; };
  int GetTrajDim() const { return n_r_; };
  bool IsActive() const { return track_at_current_state_; }

  void SaveYddotCommandSol(const Eigen::VectorXd& dv);

  // Print feedback and desired values
  void PrintFeedbackAndDesiredValues(const Eigen::VectorXd& dv);

  // Finalize and ensure that users construct OscTrackingData class
  // correctly.
  void CheckOscTrackingData();

  // For unit test
  void UpdateJAndJdotVForUnitTest(
      const Eigen::VectorXd& x_wo_spr,
      drake::systems::Context<double>& context_wo_spr);

 protected:
  int GetStateIdx() { return state_idx_; };
  void AddState(int state);

  // Feedback output, Jacobian and dJ/dt * v
  Eigen::VectorXd error_y_;
  Eigen::VectorXd error_ydot_;
  Eigen::VectorXd y_;
  Eigen::VectorXd ydot_;
  Eigen::MatrixXd J_;
  Eigen::VectorXd JdotV_;

  // Desired output
  Eigen::VectorXd y_des_;
  Eigen::VectorXd ydot_des_;
  Eigen::VectorXd yddot_des_;
  Eigen::VectorXd yddot_des_converted_;

  // Commanded acceleration after feedback terms
  Eigen::VectorXd yddot_command_;
  // Osc solution
  Eigen::VectorXd yddot_command_sol_;

  // `state_` is the finite state machine state when the tracking is enabled
  // If `state_` is empty, then the tracking is always on.
  std::vector<int> state_;

  /// OSC calculates feedback positions/velocities from `plant_w_spr_`,
  /// but in the optimization it uses `plant_wo_spr_`. The reason of using
  /// MultibodyPlant without springs is that the OSC cannot track desired
  /// acceleration instantaneously when springs exist. (relative degrees of 4)
  const drake::multibody::MultibodyPlant<double>* plant_w_spr_;
  const drake::multibody::MultibodyPlant<double>* plant_wo_spr_;

  // World frames
  const drake::multibody::BodyFrame<double>& world_w_spr_;
  const drake::multibody::BodyFrame<double>& world_wo_spr_;

 private:
  // Check if we should do tracking in the current state
  void UpdateTrackingFlag(int finite_state_machine_state);

  // Updaters of feedback output, jacobian and dJ/dt * v
  virtual void UpdateYAndError(
      const Eigen::VectorXd& x_w_spr,
      drake::systems::Context<double>& context_w_spr) = 0;
  virtual void UpdateYdotAndError(
      const Eigen::VectorXd& x_w_spr,
      drake::systems::Context<double>& context_w_spr) = 0;
  virtual void UpdateYddotDes() = 0;
  virtual void UpdateJ(const Eigen::VectorXd& x_wo_spr,
                       drake::systems::Context<double>& context_wo_spr) = 0;
  virtual void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                           drake::systems::Context<double>& context_wo_spr) = 0;

  // Finalize and ensure that users construct OscTrackingData derived class
  // correctly.
  virtual void CheckDerivedOscTrackingData() = 0;

  // Trajectory name
  std::string name_;

  // Dimension of the traj
  int n_r_;

  // PD control gains
  Eigen::MatrixXd K_p_;
  Eigen::MatrixXd K_d_;

  // Cost weights
  Eigen::MatrixXd W_;

  // Store whether or not the tracking data is active
  bool track_at_current_state_;
  int state_idx_ = 0;
};

/// ComTrackingData is used when we want to track center of mass trajectory.
class ComTrackingData final : public OscTrackingData {
 public:
  ComTrackingData(
      const std::string& name, int n_r, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr);

  //  ComTrackingData() {}  // Default constructor

  // If state is not specified, it will track COM for all states
  void AddStateToTrack(int state);

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       drake::systems::Context<double>& context_w_spr) final;
  void UpdateYdotAndError(const Eigen::VectorXd& x_w_spr,
                          drake::systems::Context<double>& context_w_spr) final;
  void UpdateYddotDes() final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   drake::systems::Context<double>& context_wo_spr) final;

  void CheckDerivedOscTrackingData() final;
};

// TaskSpaceTrackingData is still a virtual class
class TaskSpaceTrackingData : public OscTrackingData {
 public:
  TaskSpaceTrackingData(
      const std::string& name, int n_r, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr);

 protected:
  // `body_index_w_spr` is the index of the body
  // `body_index_wo_spr` is the index of the body
  // If `body_index_w_spr_` is empty, `body_index_wo_spr_` replaces it.
  std::vector<drake::multibody::BodyIndex> body_index_w_spr_;
  std::vector<drake::multibody::BodyIndex> body_index_wo_spr_;
  std::vector<const drake::multibody::BodyFrame<double>*> body_frames_w_spr_;
  std::vector<const drake::multibody::BodyFrame<double>*> body_frames_wo_spr_;
};

/// TransTaskSpaceTrackingData is used when we want to track a trajectory
/// (translational position) in the task space.

/// AddPointToTrack() should be called to specify what is the point that
/// follows the desired trajectory.

/// If users want to track the trajectory only in some states of the finite
/// state machine, they should use AddStateAndPointToTrack().
/// Also, at most one point (of the body) can follow the desired trajectory, so
/// state_ elements can not repeat, and the length of state_ must be the same as
/// pt_on_body_'s if state_ is not empty.
/// This also means that AddPointToTrack and AddStateAndPointToTrack cannot be
/// called one after another for the same TrackingData.
class TransTaskSpaceTrackingData final : public TaskSpaceTrackingData {
 public:
  TransTaskSpaceTrackingData(
      const std::string& name, int n_r, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr);

  void AddPointToTrack(
      const std::string& body_name,
      const Eigen::Vector3d& pt_on_body = Eigen::Vector3d::Zero());
  void AddStateAndPointToTrack(
      int state, const std::string& body_name,
      const Eigen::Vector3d& pt_on_body = Eigen::Vector3d::Zero());

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       drake::systems::Context<double>& context_w_spr) final;
  void UpdateYdotAndError(const Eigen::VectorXd& x_w_spr,
                          drake::systems::Context<double>& context_w_spr) final;
  void UpdateYddotDes() final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   drake::systems::Context<double>& context_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  // `pt_on_body` is the position w.r.t. the origin of the body
  std::vector<Eigen::Vector3d> pts_on_body_;
};

/// RotTaskSpaceTrackingData is used when we want to track a trajectory
/// (rotational position) in the task space. The desired position must be
/// expressed in quaternion (a 4d vector).

/// AddFrameToTrack() should be called to specify what is the frame that
/// follows the desired trajectory

/// If users want to track the trajectory only in some states of the finite
/// state machine, they should use AddStateAndFrameToTrack().
/// Also, at most one point (of the body) can follow the desired trajectory, so
/// state_ elements can not repeat, and the length of state_ must be the same as
/// frame_pose_'s if state_ is not empty.
/// This also means that AddFrameToTrack and AddStateAndFrameToTrack cannot be
/// called one after another for the same TrackingData.
class RotTaskSpaceTrackingData final : public TaskSpaceTrackingData {
 public:
  RotTaskSpaceTrackingData(
      const std::string& name, int n_r, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr);

  void AddFrameToTrack(
      const std::string& body_name,
      const Eigen::Isometry3d& frame_pose = Eigen::Isometry3d::Identity());
  void AddStateAndFrameToTrack(
      int state, const std::string& body_name,
      const Eigen::Isometry3d& frame_pose = Eigen::Isometry3d::Identity());

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       drake::systems::Context<double>& context_w_spr) final;
  void UpdateYdotAndError(const Eigen::VectorXd& x_w_spr,
                          drake::systems::Context<double>& context_w_spr) final;
  void UpdateYddotDes() final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   drake::systems::Context<double>& context_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  // frame_pose_ represents the pose of the frame (w.r.t. the body's frame)
  // which follows the desired rotation.
  std::vector<Eigen::Isometry3d> frame_pose_;
};

/// JointSpaceTrackingData is used when we want to track a trajectory
/// in the joint space.

/// AddJointToTrack() should be called to specify which joint to track.
/// Note that one instance of `JointSpaceTrackingData` allows to track 1 joint.

/// If users want to track the trajectory only in some states of the finite
/// state machine, they should use AddStateAndJointToTrack().
/// Also, at most one point (of the body) can follow the desired trajectory, so
/// state_ elements can not repeat, and the length of state_ must be the same as
/// joint_idx's if state_ is not empty.
/// This also means that AddJointToTrack and AddStateAndJointToTrack cannot be
/// called one after another for the same TrackingData.
class JointSpaceTrackingData final : public OscTrackingData {
 public:
  JointSpaceTrackingData(
      const std::string& name, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr);

  void AddJointToTrack(const std::string& joint_pos_name,
                       const std::string& joint_vel_name);
  void AddStateAndJointToTrack(int state, const std::string& joint_pos_name,
                               const std::string& joint_vel_name);

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       drake::systems::Context<double>& context_w_spr) final;
  void UpdateYdotAndError(const Eigen::VectorXd& x_w_spr,
                          drake::systems::Context<double>& context_w_spr) final;
  void UpdateYddotDes() final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   drake::systems::Context<double>& context_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  // `joint_pos_idx_wo_spr` is the index of the joint position
  // `joint_vel_idx_wo_spr` is the index of the joint velocity
  std::vector<int> joint_pos_idx_w_spr_;
  std::vector<int> joint_vel_idx_w_spr_;
  std::vector<int> joint_pos_idx_wo_spr_;
  std::vector<int> joint_vel_idx_wo_spr_;
};

// TODO(yminchen): You can probably use symbolics of drake
//  Also, drake polynomial function can potentially help you to do derivatives
class AbstractTrackingData final : public OscTrackingData {
 public:
  AbstractTrackingData(
      const std::string& name, int n_r, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr,
      OscUserDefinedPos* user_defined_pos);
  AbstractTrackingData(
      const std::string& name, int n_r, const Eigen::MatrixXd& K_p,
      const Eigen::MatrixXd& K_d, const Eigen::MatrixXd& W,
      const drake::multibody::MultibodyPlant<double>* plant_w_spr,
      const drake::multibody::MultibodyPlant<double>* plant_wo_spr,
      OscUserDefinedPos* user_defined_pos_w_spr,
      OscUserDefinedPos* user_defined_pos_wo_spr);

 private:
  void UpdateYAndError(const Eigen::VectorXd& x_w_spr,
                       drake::systems::Context<double>& context_w_spr) final;
  void UpdateYdotAndError(const Eigen::VectorXd& x_w_spr,
                  drake::systems::Context<double>& context_w_spr) final;
  void UpdateYddotDes() final;
  void UpdateJ(const Eigen::VectorXd& x_wo_spr,
               drake::systems::Context<double>& context_wo_spr) final;
  void UpdateJdotV(const Eigen::VectorXd& x_wo_spr,
                   drake::systems::Context<double>& context_wo_spr) final;

  void CheckDerivedOscTrackingData() final;

  // Compute Jacobian (qdot version instead of v!) by numerical differentiation
  Eigen::MatrixXd JacobianOfUserDefinedPos(
      const OscUserDefinedPos& user_defined_pos, Eigen::VectorXd q) const;

  // Compute the matrix for mapping global roll-pitch-yaw angular velocity to
  // quaternion derivatives
  // Ref: equation 16 of https://arxiv.org/pdf/0811.2889.pdf
  Eigen::MatrixXd WToQuatDotMap(const Eigen::Vector4d& q) const;
  Eigen::MatrixXd JwrtqdotToJwrtv(const Eigen::VectorXd& q,
                                  const Eigen::MatrixXd& Jwrtqdot) const;

  bool only_one_user_defined_pos_;

  OscUserDefinedPos* user_defined_pos_wo_spr_;
  OscUserDefinedPos* user_defined_pos_w_spr_;

  // Map position/velocity from model with spring to without spring
  Eigen::MatrixXd map_position_from_spring_to_no_spring_;
  Eigen::MatrixXd map_velocity_from_spring_to_no_spring_;

  // Step size for numerical differentiation
  // It's tuned for minimizing JdotV error:
  // (states and user-defined functions are in the osc_tracking_data_test.cc)
  //   dx_ = 1e-5 if forward differencing.
  //                 JdotV error norm ~ 1e-5, J error norm ~ 1e-5,
  //                 runtime for JdotV calculation (Cassie CoM) ~ 1.6ms
  //   dx_ = 4e-4 if central differencing.
  //                 JdotV error norm ~ 5e-8, J error norm ~ 5e-9
  //                 runtime for JdotV calculation (Cassie CoM) ~ 3.0ms
  double dx_ = 1e-5;
  // if is_forward_differencing_ = false, we use central differencing
  bool is_forward_differencing_ = true;
};

}  // namespace controllers
}  // namespace systems
}  // namespace dairlib
