#pragma once

#include "systems/framework/timestamped_vector.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/tree/multibody_tree.h"

#include <iostream>
#include <fstream>
#include <math.h>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using Eigen::Quaternion;
using Eigen::Quaterniond;
using Eigen::AngleAxis;
using Eigen::AngleAxisd;
using drake::systems::LeafSystem;
using drake::systems::Context;
using drake::multibody::MultibodyPlant;
using drake::multibody::Frame;

namespace dairlib{
namespace systems{

// PD Controller for end effector position -- Written specifically for
// Kuka Iiwa robot arm, but will work for other robot arms with a bit of work.
// Takes in desired end effector pos (3-vector) and orientation (quaternion),
// outputs appropriate desired velocity (twist) as 6-vector: [angular | linear]

class EndEffectorPositionController : public LeafSystem<double> {
 public:
   // Constructor
   EndEffectorPositionController(const MultibodyPlant<double>& plant,
                                 std::string ee_frame_name,
                                 Eigen::Vector3d ee_contact_frame,
                                 double k_p, double k_omega,
                                 double max_linear_vel, double max_angular_vel, double pos_tolerance, double rot_tolerance);

   // Getter methods for each of the individual input/output ports.
   const drake::systems::InputPort<double>& get_joint_pos_input_port() const {
     return this->get_input_port(joint_position_measured_port_);
   }
   const drake::systems::InputPort<double>& get_endpoint_pos_input_port() const {
     return this->get_input_port(endpoint_position_commanded_port_);
   }
   const drake::systems::InputPort<double>& get_endpoint_vel_input_port() const {
       return this->get_input_port(endpoint_velocity_commanded_port_);
   }
   const drake::systems::InputPort<double>& get_endpoint_orient_input_port() const {
     return this->get_input_port(endpoint_orientation_commanded_port_);
   }
   const drake::systems::OutputPort<double>& get_endpoint_cmd_output_port() const {
     return this->get_output_port(endpoint_twist_cmd_output_port_);
   }

   const drake::systems::OutputPort<double>& get_failure_output_port() const {
     return this->get_output_port(failure_output_port_);
   }

 private:
   // Callback method called when declaring output port of the system.
   // Twist combines linear and angular velocities.
   void CalcOutputTwist(const Context<double> &context,
                        BasicVector<double>* output) const;

    void failureCalc(const Context<double> &context,
                    bool* failure) const;

    Eigen::VectorXd clampToNorm(Eigen::VectorXd v, double norm, std::string msg) const;

   const MultibodyPlant<double>& plant_;
   const Frame<double>& plant_world_frame_;
   Eigen::Vector3d ee_contact_frame_;
   const Frame<double>& ee_joint_frame_;


   double k_p_;
   double k_omega_;
   double max_linear_vel_;
   double max_angular_vel_;
   double pos_tolerance_;
   double rot_tolerance_;

   int joint_position_measured_port_;
   int endpoint_position_commanded_port_;
   int endpoint_velocity_commanded_port_;
   int endpoint_orientation_commanded_port_;
   int endpoint_twist_cmd_output_port_;
   int failure_output_port_;

   



};

}
}
