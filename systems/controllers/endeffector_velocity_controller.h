#pragma once

#include "common/find_resource.h"
#include "systems/framework/timestamped_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
// #include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"

#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::multibody::MultibodyPlant;
using drake::multibody::Frame;

namespace dairlib{
namespace systems{

// KukaIiwaVelocityController extends LeafSystem, which is basically
// a 'block' like you might see in simulink with inputs and outputs.
// In this case, KukaIiwaVelocityController will take in desired velocities,
// q, q_dot, and output an appropriate torque
// \Tau = jacobian.transpose x K x desired_accelerations
class EndEffectorVelocityController : public LeafSystem<double> {
  public:
    // Constructor
    EndEffectorVelocityController(const MultibodyPlant<double>& plant,
                                  std::string ee_frame_name,
                                  Eigen::Vector3d ee_contact_frame,
                                  double k_d, double k_r,
                                  double joint_torque_limit,
                                  Eigen::DiagonalMatrix<double, 7> joint_torque_mult,
                                  double null_space_grad_gain,
                                  double joint_lim_grad_d );

    // Getter methods for each of the individual input/output ports.
    const drake::systems::InputPort<double>& get_joint_pos_input_port() const {
      return this->get_input_port(_joint_position_measured_port);
    }
    const drake::systems::InputPort<double>& get_joint_vel_input_port() const {
      return this->get_input_port(_joint_velocity_measured_port);
    }
    const drake::systems::InputPort<double>& get_endpoint_twist_input_port() const {
      return this->get_input_port(_endpoint_twist_commanded_port);
    }
    const drake::systems::OutputPort<double>& get_endpoint_torque_output_port() const{
      return this->get_output_port(_endpoint_torque_output_port);
    }
    const drake::systems::OutputPort<double>& get_failure_output_port() const {
     return this->get_output_port(failure_output_port_);
   }

  private:
    // The callback called when declaring the output port of the system.
    // The 'output' vector is set in place and then passed out.
    // Think a simulink system.
    void CalcOutputTorques(const Context<double>& context,
                         BasicVector<double>* output) const;
    
    void failureCalc(const Context<double> &context,
                      bool* failure) const;

    const MultibodyPlant<double>& _plant;
    int _num_joints;
    const Frame<double>& _ee_joint_frame;
    

    //ports
    int _joint_position_measured_port;
    int _joint_velocity_measured_port;
    int _endpoint_twist_commanded_port;
    int _endpoint_torque_output_port;
    int failure_output_port_;



    //controller params
    Eigen::Vector3d _ee_contact_frame;
    double _k_d;
    double _k_r;
    double _joint_torque_limit;
    Eigen::DiagonalMatrix<double, 7> _joint_torque_mult;
    double _null_space_grad_gain;
    double _joint_lim_grad_d;

    mutable bool is_failing = false;

};


} // namespace systems
} // namespace dairlib
