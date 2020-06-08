#include "systems/trajectory_optimization/dircon/dircon.h"

#include "multibody/kinematic/kinematic_constraints.h"
#include "systems/trajectory_optimization/dircon/dircon_opt_constraints.h"

#include "multibody/multibody_utils.h"

namespace dairlib {
namespace systems {
namespace trajectory_optimization {

using drake::multibody::MultibodyPlant;
using drake::solvers::MathematicalProgramResult;
using drake::solvers::VectorXDecisionVariable;
using drake::symbolic::Expression;
using drake::systems::Context;
using drake::trajectories::PiecewisePolynomial;
using drake::VectorX;

using Eigen::MatrixXd;
using Eigen::VectorXd;

using multibody::KinematicPositionConstraint;
using multibody::KinematicVelocityConstraint;
using multibody::KinematicAccelerationConstraint;

template <typename T>
Dircon<T>::Dircon(const DirconModeSequence<T>& mode_sequence)
  : Dircon<T>({}, &mode_sequence, mode_sequence.plant(),
        mode_sequence.count_knotpoints()) {}

template <typename T>
Dircon<T>::Dircon(DirconMode<T>* mode)
  : Dircon<T>(std::make_unique<DirconModeSequence<T>>(mode),
        nullptr,
        mode->plant(),
        mode->num_knotpoints()) {}

/// Private constructor. Determines which DirconModeSequence was provided,
/// a locally owned unique_ptr or an externally owned const reference
template <typename T>
Dircon<T>::Dircon(std::unique_ptr<DirconModeSequence<T>> my_sequence,
    const DirconModeSequence<T>* ext_sequence,
    const MultibodyPlant<T>& plant,
    int num_knotpoints)
    : drake::systems::trajectory_optimization::MultipleShooting(
          plant.num_actuators(),
          plant.num_positions() + plant.num_velocities(),
          num_knotpoints, 1e-8, 1e8),
      my_sequence_(std::move(my_sequence)),
      plant_(plant),
      mode_sequence_(ext_sequence ? *ext_sequence : *my_sequence_),
      contexts_(num_modes()),
      mode_start_(num_modes()) {
  // Loop over all modes
  for (int i_mode = 0; i_mode < num_modes(); i_mode++) {    
    const auto& mode = mode_sequence_.mode(i_mode);

    // Identify starting index for this mode, accounting for shared knotpoints
    if (i_mode == 0) {
      mode_start_[i_mode] = 0;
    } else {
      mode_start_[i_mode] = mode_start_[i_mode-1] +
          mode_length(i_mode - 1) - 1;
    }
    
    //
    // Set constraints on timesteps
    //
    double min_dt = mode.min_T() / (mode.num_knotpoints() - 1);
    double max_dt = mode.max_T() / (mode.num_knotpoints() - 1);
    // set timestep bounds
    for (int j = 0; j < mode.num_knotpoints() - 1; j++) {
      AddBoundingBoxConstraint(min_dt, max_dt,
          timestep(mode_start_[i_mode] + j));
    }
    for (int j = 0; j < mode.num_knotpoints() - 2; j++) {
      // all timesteps must be equal
      AddLinearConstraint(timestep(mode_start_[i_mode] + j) ==
                          timestep(mode_start_[i_mode] + j + 1));
    }

    //
    // Create new decision variables
    //
    force_vars_.push_back(NewContinuousVariables(
        mode.evaluators().count_full() * mode.num_knotpoints(),
        "lambda[" + std::to_string(i_mode) + "]"));
    collocation_force_vars_.push_back(NewContinuousVariables(
        mode.evaluators().count_full() * (mode.num_knotpoints() - 1),
        "lambda_c[" + std::to_string(i_mode) + "]"));
    collocation_slack_vars_.push_back(NewContinuousVariables(
        mode.evaluators().count_full() * (mode.num_knotpoints() - 1),
        "gamma[" + std::to_string(i_mode) + "]"));

    // Bound collocation slack variables to avoid numerical issues
    AddBoundingBoxConstraint(-.1, .1, collocation_slack_vars_.at(i_mode));

    // quaternion_slack_vars_ (slack variables used to scale quaternion norm to
    // 1 in the dynamic constraints)
    int num_quat = multibody::QuaternionStartIndices(plant_).size();
    quaternion_slack_vars_.push_back(NewContinuousVariables(
        num_quat * (mode.num_knotpoints() - 1),
        "quat_slack[" + std::to_string(i_mode) + "]"));

    // Bound quaternion slack variables to avoid false full rotations
    AddBoundingBoxConstraint(-.1, .1, quaternion_slack_vars_.at(i_mode));

    // Impulse and post-impact variables
    if (i_mode > 0) {
      impulse_vars_.push_back(NewContinuousVariables(
          mode.evaluators().count_full(),
          "impulse[" + std::to_string(i_mode) + "]"));
      v_post_impact_vars_.push_back(NewContinuousVariables(
          plant_.num_velocities(),
          "v_p[" + std::to_string(i_mode) + "]"));
    }

    // Constraint offset variables for relative constraints
    offset_vars_.push_back(NewContinuousVariables(
        mode.num_relative_constraints(),
        "rel_offset[" + std::to_string(i_mode) + "]"));

    //
    // Create context elements for knot points
    //
    for (int j = 0; j < mode.num_knotpoints(); j++) {
      contexts_[i_mode].push_back(std::move(plant_.CreateDefaultContext()));
    }

    //
    // Create and add collocation constraints
    //
    for (int j = 0; j < mode.num_knotpoints() - 1; j++) {
      auto constraint = std::make_shared<DirconCollocationConstraint<T>>(
          plant_, mode.evaluators(), contexts_[i_mode].at(j).get(),
          contexts_[i_mode].at(j+1).get(), i_mode, j);
      constraint->SetConstraintScaling(mode.GetDynamicsScale());
      AddConstraint(constraint,
          { timestep(mode_start_[i_mode] + j),
            state_vars(i_mode, j),
            state_vars(i_mode, j + 1),
            input_vars(i_mode, j),
            input_vars(i_mode, j + 1),
            force_vars(i_mode, j),
            force_vars(i_mode, j + 1),
            collocation_force_vars(i_mode, j),
            collocation_slack_vars(i_mode, j),
            quaternion_slack_vars(i_mode, j) });
    }

    //
    // Create and add kinematic constraints
    //
    for (int j = 0; j < mode.num_knotpoints(); j++) {
      // Position constraints if type is All
      if (mode.get_constraint_type(j) == DirconKinConstraintType::kAll) {
        auto pos_constraint = std::make_shared<KinematicPositionConstraint<T>>(
            plant_, mode.evaluators(),
            VectorXd::Zero(mode.evaluators().count_active()),
            VectorXd::Zero(mode.evaluators().count_active()),
            mode.relative_constraints(),
            contexts_[i_mode].at(j).get(),
            "kinematic_position[" + std::to_string(i_mode) + "]["
                + std::to_string(j) + "]");
        pos_constraint->SetConstraintScaling(mode.GetKinPositionScale());
        AddConstraint(pos_constraint,
            { state_vars(i_mode, j).head(plant_.num_positions()),
              offset_vars(i_mode) });
      }

      // Velocity constraints if type is not acceleration only
      if (mode.get_constraint_type(j) != DirconKinConstraintType::kAccelOnly) {
        auto vel_constraint = std::make_shared<KinematicVelocityConstraint<T>>(
            plant_, mode.evaluators(),
            VectorXd::Zero(mode.evaluators().count_active()),
            VectorXd::Zero(mode.evaluators().count_active()),
            contexts_[i_mode].at(j).get(),
            "kinematic_velocity[" + std::to_string(i_mode) + "]["
                + std::to_string(j) + "]");
        vel_constraint->SetConstraintScaling(mode.GetKinVelocityScale());
        AddConstraint(vel_constraint, state_vars(i_mode, j));
      }

      // Acceleration constraints (always)
      auto accel_constraint =
          std::make_shared<KinematicAccelerationConstraint<T>>(
            plant_, mode.evaluators(),
            VectorXd::Zero(mode.evaluators().count_active()),
            VectorXd::Zero(mode.evaluators().count_active()),
            contexts_[i_mode].at(j).get(),
            "kinematic_acceleration[" + std::to_string(i_mode) + "]["
                + std::to_string(j) + "]");
      accel_constraint->SetConstraintScaling(mode.GetKinAccelerationScale());
      AddConstraint(accel_constraint,
          { state_vars(i_mode, j),
            input_vars(i_mode, j),
            force_vars(i_mode, j) });
    }

    //
    // Create and add impact constraints
    //
    if (i_mode > 0) {
      // Use pre-impact context
      int pre_impact_index = mode_length(i_mode - 1) - 1;
      auto impact_constraint = 
          std::make_shared<DirconImpactConstraint<T>>(
            plant_, mode.evaluators(),
            contexts_[i_mode - 1].back().get(),
            "impact[" + std::to_string(i_mode) + "]");
      impact_constraint->SetConstraintScaling(mode.GetImpactScale());
      AddConstraint(impact_constraint,
          { state_vars(i_mode - 1, pre_impact_index),
            impulse_vars(i_mode - 1),
            post_impact_velocity_vars(i_mode - 1) });
    }

    //
    // Create and add quaternion constraints
    //
    auto quaternion_constraint = 
      std::make_shared<QuaternionNormConstraint<T>>();
    for (int j = 0; j < mode.num_knotpoints(); j++) {
      if (!mode.IsSkipQuaternion(j)) {
        auto start_indices = multibody::QuaternionStartIndices(plant_);
        for (auto start_index : start_indices) {
          AddConstraint(quaternion_constraint,
              state_vars(i_mode, j).segment(start_index, 4));
        }
      }
    }

    ///
    /// Add friction cone constraints to force variables
    ///
    /// TODO: hard-coding number of frictional faces, but this could be an
    /// option, along with a choice of which friction cone constraint to use.
    int num_faces = 4;
    for (int k = 0; k < mode.evaluators().num_evaluators(); k++) {
      const auto& e = mode.evaluators().get_evaluator(k);
      auto force_constraint = e.CreateLinearFrictionConstraint(mode.mu(),
          num_faces);
      if (force_constraint) {
        // Add to knot point forces
        for (int j = 0; j < mode.num_knotpoints(); j++) {
          AddConstraint(force_constraint,
              force_vars(i_mode, j).segment(
                  mode.evaluators().evaluator_full_start(k),
                  e.num_full()));
        }

        if (i_mode > 0) {
          // Add to impulse variables
          AddConstraint(force_constraint,
              impulse_vars(i_mode - 1).segment(
                  mode.evaluators().evaluator_full_start(k),
                  e.num_full()));
        }
      }
    }

    // Add regularization cost on force
    int n_l = mode.evaluators().count_full() * mode.num_knotpoints();
    AddQuadraticCost(
        mode.get_force_regularization() * MatrixXd::Identity(n_l, n_l),
        VectorXd::Zero(n_l),
        force_vars_.at(i_mode));
  }
}

///
/// Getters for decision variables
///
template <typename T>
const VectorXDecisionVariable Dircon<T>::force_vars(int mode_index,
    int knotpoint_index) const {
  const auto& mode = mode_sequence_.mode(mode_index);
  return force_vars_.at(mode_index).segment(
      knotpoint_index * mode.evaluators().count_full(),
      mode.evaluators().count_full());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::collocation_force_vars(int mode_index,
    int knotpoint_index) const {
  const auto& mode = mode_sequence_.mode(mode_index);
  return collocation_force_vars_.at(mode_index).segment(
      knotpoint_index * mode.evaluators().count_full(),
      mode.evaluators().count_full());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::collocation_slack_vars(int mode_index,
    int knotpoint_index) const {
  const auto& mode = mode_sequence_.mode(mode_index);
  return collocation_slack_vars_.at(mode_index).segment(
      knotpoint_index * mode.evaluators().count_full(),
      mode.evaluators().count_full());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::state_vars(int mode_index,
    int knotpoint_index) const {
  // If first knot of a mode after the first, use post impact velocity variables
  if (knotpoint_index == 0 && mode_index > 0) {
    VectorXDecisionVariable
        ret(plant_.num_positions() + plant_.num_velocities());
    ret << x_vars().segment(mode_start_[mode_index] 
        * (plant_.num_positions() + plant_.num_velocities()),
           plant_.num_positions()),
        post_impact_velocity_vars(mode_index - 1);
    return ret;
  } else {
    return x_vars().segment((mode_start_[mode_index] + knotpoint_index)
            * (plant_.num_positions() + plant_.num_velocities()),
        plant_.num_positions() + plant_.num_velocities());
  }
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::input_vars(int mode_index,
    int knotpoint_index) const {
  return u_vars().segment((mode_start_[mode_index] + knotpoint_index)
      * plant_.num_actuators(), plant_.num_actuators());
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::quaternion_slack_vars(int mode_index,
    int knotpoint_index) const {
  int num_quat = multibody::QuaternionStartIndices(plant_).size();
  return quaternion_slack_vars_.at(mode_index).segment(
      num_quat * knotpoint_index, num_quat);
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::offset_vars(int mode_index) const {
  return offset_vars_.at(mode_index);
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::post_impact_velocity_vars(
    int mode_transition_index) const {
  return v_post_impact_vars_.at(mode_transition_index);
}

template <typename T>
const VectorXDecisionVariable Dircon<T>::impulse_vars(
    int mode_transition_index) const {
  return impulse_vars_.at(mode_transition_index);
}

template <typename T>
void Dircon<T>::CreateVisualizationCallback(std::string model_file,
    std::vector<unsigned int> poses_per_mode, std::string weld_frame_to_world) {
  DRAKE_DEMAND(!callback_visualizer_);  // Cannot be set twice
  DRAKE_DEMAND(poses_per_mode.size() == (uint) num_modes());

  // Count number of total poses, start and finish of every mode
  int num_poses = num_modes() + 1;
  for (int i = 0; i < num_modes(); i++) {
    DRAKE_DEMAND(poses_per_mode.at(i) == 0 ||  (poses_per_mode.at(i) + 2
        <= (uint) mode_length(i)));
    num_poses += poses_per_mode.at(i);
  }

  // Assemble variable list
  drake::solvers::VectorXDecisionVariable vars(
      num_poses * plant_.num_positions());

  int index = 0;
  for (int i = 0; i < num_modes(); i++) {
    // Set variable block, extracting positions only
    vars.segment(index * plant_.num_positions(), plant_.num_positions()) = 
        state_vars(i, 0).head(plant_.num_positions());
    index++;

    for (uint j = 0; j < poses_per_mode.at(i); j++) {
      // The jth element in mode i, counting in between the start/end poses
      int modei_index = (j + 1) * (mode_length(i) - 1)
          /(poses_per_mode.at(i) + 1);

      vars.segment(index * plant_.num_positions(), plant_.num_positions()) = 
          state_vars(i, modei_index).head(plant_.num_positions());
      index++;
    } 
  }

  // Final state
  auto last_mode = mode_sequence_.mode(num_modes() - 1);
  vars.segment(index * plant_.num_positions(), plant_.num_positions()) = 
      state_vars(num_modes() - 1,
          last_mode.num_knotpoints() - 1).head(plant_.num_positions());

  // Create visualizer
  callback_visualizer_ = std::make_unique<multibody::MultiposeVisualizer>(
    model_file, num_poses, weld_frame_to_world);

  // Callback lambda function
  auto my_callback = [this, num_poses](const Eigen::Ref<const VectorXd>& vars) {
    VectorXd vars_copy = vars;
    Eigen::Map<MatrixXd> states(vars_copy.data(), this->plant_.num_positions(),
        num_poses);

    this->callback_visualizer_->DrawPoses(states);
  };

  AddVisualizationCallback(my_callback, vars);
}

template <typename T>
void Dircon<T>::CreateVisualizationCallback(std::string model_file,
      unsigned int num_poses, std::string weld_frame_to_world) {
  // Check that there is an appropriate number of poses
  DRAKE_DEMAND(num_poses >= (uint) num_modes() + 1);
  DRAKE_DEMAND(num_poses <= (uint) N());

  // sum of mode lengths, excluding ends
  int mode_sum = 0;
  for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
    auto mode = mode_sequence_.mode(i_mode);
    if (mode.num_knotpoints() > 2) {
      mode_sum += mode.num_knotpoints() - 2;
    }
  }

  // Split up num_poses per modes, rounding down
  // If NP is the total number of  poses to visualize, excluding ends (interior)
  //   S is mode_sum, the total number of the interior knot points
  //   and N the number of interior knot points for a particular mode, then 
  //
  //   poses = (NP * N )/S
  unsigned int num_poses_without_ends =
      num_poses - num_modes() - 1;
  unsigned int assigned_sum = 0;
  std::vector<unsigned int> num_poses_per_mode;
  for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
    if (mode_length(i_mode) > 2) {
      unsigned int mode_count = (num_poses_without_ends
            * (mode_length(i_mode) - 2)) / mode_sum;
      num_poses_per_mode.push_back(mode_count);
      assigned_sum += mode_count;
    } else {
      num_poses_per_mode.push_back(0);
    }
  }

  // Need to add back in the fractional bits, using the largest fractions
  while (assigned_sum < num_poses_without_ends) {
    int largest_mode = -1;
    double value = 0;
    for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
      double fractional_value = num_poses_without_ends
          * (mode_length(i_mode) - 2)
          - num_poses_per_mode.at(i_mode) * mode_sum;

      if (fractional_value > value) {
        value = fractional_value;
        largest_mode = i_mode;
      }
    }

    num_poses_per_mode.at(largest_mode)++;
    assigned_sum++;
  }

  CreateVisualizationCallback(model_file, num_poses_per_mode,
      weld_frame_to_world);
}

template <typename T>
void Dircon<T>::CreateVisualizationCallback(std::string model_file,
      std::string weld_frame_to_world) {
  CreateVisualizationCallback(model_file, N(), weld_frame_to_world);
}

template <typename T>
VectorX<Expression> Dircon<T>::SubstitutePlaceholderVariables(
    const VectorX<Expression>& f, int interval_index) const {
  VectorX<Expression> ret(f.size());
  for (int i = 0; i < f.size(); i++) {
    ret(i) =
        MultipleShooting::SubstitutePlaceholderVariables(f(i), interval_index);
  }
  return ret;
}

// TODO: need to configure this to handle the hybrid discontinuities properly
template <typename T>
void Dircon<T>::DoAddRunningCost(const drake::symbolic::Expression& g) {
  // Trapezoidal integration:
  //    sum_{i=0...N-2} h_i/2.0 * (g_i + g_{i+1}), or
  // g_0*h_0/2.0 + [sum_{i=1...N-2} g_i*(h_{i-1} + h_i)/2.0] +
  // g_{N-1}*h_{N-2}/2.0.

  // Here, we add the cost using symbolic expression. The expression is a
  // polynomial of degree 3 which Drake can handle, although the
  // documentation says it only supports up to second order.
  AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, 0) * h_vars()(0) /
          2);
  for (int i = 1; i <= N() - 2; i++) {
    AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, i) *
            (h_vars()(i - 1) + h_vars()(i)) / 2);
  }
  AddCost(MultipleShooting::SubstitutePlaceholderVariables(g, N() - 1) *
          h_vars()(N() - 2) / 2);
}

template <typename T>
PiecewisePolynomial<double> Dircon<T>::ReconstructInputTrajectory(
    const MathematicalProgramResult& result) const {
  Eigen::VectorXd times = GetSampleTimes(result);
  Eigen::MatrixXd inputs(plant_.num_actuators(), N());
  for (int i = 0; i < N(); i++) {
    inputs.col(i) = result.GetSolution(input(i));
  }
  return PiecewisePolynomial<double>::FirstOrderHold(times, inputs);
}

// TODO(posa)
// need to configure this to handle the hybrid discontinuities properly
template <typename T>
PiecewisePolynomial<double> Dircon<T>::ReconstructStateTrajectory(
    const MathematicalProgramResult& result) const {
  VectorXd times_all(GetSampleTimes(result));
  VectorXd times(N() + num_modes() - 1);

  MatrixXd states(num_states(), N() + num_modes() - 1);
  MatrixXd inputs(num_inputs(), N() + num_modes() - 1);
  MatrixXd derivatives(num_states(), N() + num_modes() - 1);

  for (int i = 0; i < num_modes(); i++) {
    for (int j = 0; j < mode_length(i); j++) {
      int k = mode_start_[i] + j + i;
      times(k) = times_all(mode_start_[i] + j);

      // False timestep to match velocities
      if (i > 0 && j == 0) {
        times(k) += +1e-6;
      }
      VectorX<T> xk = result.GetSolution(state_vars(i, j));
      VectorX<T> uk = result.GetSolution(input_vars(i, j));
      states.col(k) = drake::math::DiscardGradient(xk);
      inputs.col(k) = drake::math::DiscardGradient(uk);
      auto context = multibody::createContext<T>(plant_, xk, uk);
      auto xdot = mode_sequence_.mode(i).evaluators().
          CalcTimeDerivativesWithForce(*context,
              result.GetSolution(force_vars(i, j)));
      derivatives.col(k) = drake::math::DiscardGradient(xdot);
    }
  }
  return PiecewisePolynomial<double>::CubicHermite(times, states, derivatives);
}

template <typename T>
void Dircon<T>::SetInitialForceTrajectory(
    int mode_index, const PiecewisePolynomial<double>& traj_init_l,
    const PiecewisePolynomial<double>& traj_init_lc,
    const PiecewisePolynomial<double>& traj_init_vc) {
  const auto& mode = mode_sequence_.mode(mode_index);
  double start_time = 0;
  double h;
  if (timesteps_are_decision_variables())
    h = GetInitialGuess(h_vars()[0]);
  else
    h = fixed_timestep();

  VectorXd guess_force(force_vars_[mode_index].size());
  if (traj_init_l.empty()) {
    guess_force.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode.num_knotpoints(); ++i) {
      guess_force.segment(mode.evaluators().count_full() * i,
                          mode.evaluators().count_full()) =
          traj_init_l.value(start_time + i * h);
    }
  }
  SetInitialGuess(force_vars_[mode_index], guess_force);

  VectorXd guess_collocation_force(collocation_force_vars_[mode_index].size());
  if (traj_init_lc.empty()) {
    guess_collocation_force.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode.num_knotpoints() - 1; ++i) {
      guess_collocation_force.segment(
          mode.evaluators().count_full() * i,
          mode.evaluators().count_full()) =
          traj_init_lc.value(start_time + (i + 0.5) * h);
    }
  }

  SetInitialGuess(collocation_force_vars_[mode_index], guess_collocation_force);

  VectorXd guess_collocation_slack(collocation_slack_vars_[mode_index].size());
  if (traj_init_vc.empty()) {
    guess_collocation_slack.fill(0);  // Start with 0
  } else {
    for (int i = 0; i < mode.num_knotpoints() - 1; ++i) {
      guess_collocation_slack.segment(
          mode.evaluators().count_full() * i,
          mode.evaluators().count_full()) =
          traj_init_vc.value(start_time + (i + 0.5) * h);
    }
  }

  // call superclass method
  SetInitialGuess(collocation_slack_vars_[mode_index], guess_collocation_slack);
}

template <typename T>
int Dircon<T>::num_modes() const {
  return mode_sequence_.num_modes();
}

template <typename T>
int Dircon<T>::mode_length(int mode_index) const {
  return mode_sequence_.mode(mode_index).num_knotpoints();
}

template <typename T>
void Dircon<T>::ScaleTimeVariables(double scale) {
  for (int i = 0; i < h_vars().size(); i++) {
    this->SetVariableScaling(h_vars()(i), scale);
  }
}

template <typename T>
void Dircon<T>::ScaleQuaternionSlackVariables(double scale) {
  DRAKE_DEMAND(multibody::isQuaternion(plant_));
  for (int i_mode = 0; i_mode < num_modes(); i_mode++) {
    for (int j = 0; j < mode_length(i_mode) - 1; j++) {
      const auto& vars = quaternion_slack_vars(i_mode, j);
      for (int k = 0; k < vars.size(); k++) {
        this->SetVariableScaling(vars(k), scale);
      }
    }
  }
}

template <typename T>
void Dircon<T>::ScaleStateVariable(int state_index, double scale) {
  DRAKE_DEMAND(0 <= state_index &&
      state_index < plant_.num_positions() + plant_.num_velocities());

  // x_vars_ in MathematicalProgram
  for (int j_knot = 0; j_knot < N(); j_knot++) {
    auto vars = this->state(j_knot);
    this->SetVariableScaling(vars(state_index), scale);
  }

  // v_post_impact_vars_
  if (state_index > plant_.num_positions()) {
    for (int mode = 0; mode < num_modes() - 1; mode++) {
      auto vars = post_impact_velocity_vars(mode);
      this->SetVariableScaling(vars(state_index - plant_.num_positions()),
          scale);
    }
  }
}

template <typename T>
void Dircon<T>::ScaleInputVariable(int input_index, double scale) {
  DRAKE_DEMAND((0 <= input_index) && (input_index < plant_.num_actuators()));

  // u_vars_ in MathematicalProgram
  for (int j_knot = 0; j_knot < N(); j_knot++) {
    auto vars = this->input(j_knot);
    this->SetVariableScaling(vars(input_index), scale);
  }
}

template <typename T>
void Dircon<T>::ScaleForceVariable(int mode_index, int force_index, double scale) {
  DRAKE_DEMAND((0 <= mode_index) && (mode_index < num_modes()));
  int n_lambda = mode_sequence_.mode(mode_index).evaluators().count_full();
  DRAKE_DEMAND((0 <= force_index) && (force_index < n_lambda));

  // Force at knot points
  for (int j = 0; j < mode_length(mode_index); j++) {
    this->SetVariableScaling(force_vars(mode_index, j)(force_index), scale);
  }
  // Force at collocation pints
  for (int j = 0; j < mode_length(mode_index) - 1; j++) {
    this->SetVariableScaling(
        collocation_force_vars(mode_index, j)(force_index), scale);
  }
}

template <typename T>
void Dircon<T>::ScaleImpulseVariable(int mode_index, int impulse_index,
    double scale) {
  DRAKE_DEMAND((0 <= mode_index) && (mode_index < num_modes() - 1));
  int n_impulse = mode_sequence_.mode(mode_index).evaluators().count_full();
  DRAKE_DEMAND((0 <= impulse_index) && (impulse_index < n_impulse));

  this->SetVariableScaling(impulse_vars(mode_index)(impulse_index), scale);
}

template <typename T>
void Dircon<T>::ScaleKinConstraintSlackVariable(int mode_index, int slack_index,
    double scale) {
  DRAKE_DEMAND((0 <= mode_index) && (mode_index < num_modes() - 1));
  int n_lambda = mode_sequence_.mode(mode_index).evaluators().count_full();
  DRAKE_DEMAND(slack_index < n_lambda);

  for (int j = 0; j < mode_length(mode_index) - 1; j++) {
    this->SetVariableScaling(collocation_slack_vars(mode_index, j)(slack_index),
        scale);
  }
}

template <typename T>
void Dircon<T>::ScaleStateVariables(std::vector<int> index_list,
    double scale) {
  for (const auto& idx : index_list) {
    ScaleStateVariable(idx, scale);
  }
}

template <typename T>
void Dircon<T>::ScaleInputVariables(std::vector<int> index_list,
    double scale) {
  for (const auto& idx : index_list) {
    ScaleInputVariable(idx, scale);
  }
}

template <typename T>
void Dircon<T>::ScaleForceVariables(int mode, std::vector<int> index_list,
    double scale) {
  for (const auto& idx : index_list) {
    ScaleForceVariable(mode, idx, scale);
  }
}

template <typename T>
void Dircon<T>::ScaleImpulseVariables(int mode, std::vector<int> index_list,
    double scale) {
  for (const auto& idx : index_list) {
    ScaleImpulseVariable(mode, idx, scale);
  }
}

template <typename T>
void Dircon<T>::ScaleKinConstraintSlackVariables(
    int mode, std::vector<int> index_list, double scale) {
  for (const auto& idx : index_list) {
    ScaleKinConstraintSlackVariable(mode, idx, scale);
  }
}

}  // namespace trajectory_optimization
}  // namespace systems
}  // namespace dairlib

DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_NONSYMBOLIC_SCALARS(
    class ::dairlib::systems::trajectory_optimization::Dircon)