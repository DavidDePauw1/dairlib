#include "systems/controllers/endeffector_velocity_controller.h"

namespace dairlib{
namespace systems{

EndEffectorVelocityController::EndEffectorVelocityController(
    const MultibodyPlant<double>& plant, std::string ee_frame_name,
    Eigen::Vector3d ee_contact_frame, double k_d, double k_r,
    double joint_torque_limit) : plant_(plant),
    num_joints_(plant_.num_positions()),
    ee_joint_frame_(plant_.GetFrameByName(ee_frame_name)){

  // Set up this block's input and output ports
  // Input port values will be accessed via EvalVectorInput() later
  joint_position_measured_port_ = this->DeclareVectorInputPort(
      "joint_position_measured", BasicVector<double>(num_joints_)).get_index();
  joint_velocity_measured_port_ = this->DeclareVectorInputPort(
      "joint_velocity_measured", BasicVector<double>(num_joints_)).get_index();
  endpoint_twist_commanded_port_ = this->DeclareVectorInputPort(
      "endpoint_twist_commanded", BasicVector<double>(6)).get_index();

  // Note that this function contains a pointer to the callback function below.
  endpoint_torque_output_port_ = this->DeclareVectorOutputPort(
      BasicVector<double>(num_joints_),
      &EndEffectorVelocityController::CalcOutputTorques).get_index();

  ee_contact_frame_ = ee_contact_frame;
  k_d_ = k_d;
  k_r_ = k_r;
  joint_torque_limit_ = joint_torque_limit;
}

// Callback for DeclareVectorInputPort. No return value.
// The parameter 'output' is the output.
// This function is called many times a second.
void EndEffectorVelocityController::CalcOutputTorques(
  const Context<double>& context, BasicVector<double>* output) const {
  // We read the above input ports with EvalVectorInput
  // The purpose of CopyToVector().head(NUM_JOINTS) is to remove the timestamp
  // from the vector input ports
  const BasicVector<double>* q_timestamped =
      (BasicVector<double>*) this->EvalVectorInput(
          context, joint_position_measured_port_);
  auto q = q_timestamped->get_value();

  const BasicVector<double>* q_dot_timestamped =
      (BasicVector<double>*) this->EvalVectorInput(
          context, joint_velocity_measured_port_);
  auto q_dot = q_dot_timestamped->get_value();

  const BasicVector<double>* twist_desired_timestamped =
      (BasicVector<double>*) this->EvalVectorInput(
          context, endpoint_twist_commanded_port_);
  auto twist_desired = twist_desired_timestamped->get_value();

  const std::unique_ptr<Context<double>> plant_context =
      plant_.CreateDefaultContext();
  plant_.SetPositions(plant_context.get(), q);
  plant_.SetVelocities(plant_context.get(), q_dot);

  // Calculating the jacobian of the kuka arm
  Eigen::MatrixXd J(6, num_joints_);
  plant_.CalcJacobianSpatialVelocity(
      *plant_context,drake::multibody::JacobianWrtVariable::kV, ee_joint_frame_, ee_contact_frame_, plant_.world_frame(), plant_.world_frame(),
      &J);
  Eigen::MatrixXd Jt = J.transpose();

  // Using the jacobian, calculating the actual current velocities of the arm
  MatrixXd twist_actual = J * q_dot;

  // Gains are placed in a diagonal matrix
  Eigen::DiagonalMatrix<double, 6> gains(6);
  gains.diagonal() << k_r_, k_r_, k_r_, k_d_, k_d_, k_d_;

  // Calculating the error
  MatrixXd error = gains * (twist_desired - twist_actual);

  VectorXd tm(num_joints_); //= Eigen::DiagonalMatrix<double>(7, 7);
  tm << 1, 0.1, 1, 0.1, 1, 2, 1;

  // Multiplying J^t x force to get torque outputs
  VectorXd torques(num_joints_);
  VectorXd commandedTorques(num_joints_);

  torques = J.transpose() * error;

  // Calculating Mass Matrix
  Eigen::MatrixXd H(plant_.num_positions(), plant_.num_positions());
  plant_.CalcMassMatrixViaInverseDynamics(*plant_context.get(), &H);
  Eigen::MatrixXd Hi = H.inverse();

  double alpha = 0.9;

  Eigen::MatrixXd T = (alpha * Eigen::MatrixXd::Identity(7, 7) + (1-alpha)*Hi).inverse();
  Eigen::MatrixXd T2 = T * T;
  commandedTorques =  Hi*Jt * (J * Hi * Jt).inverse() * (error);

//------------testing-------------
//   std::cout << "sanity test" << std::endl;
  Eigen::MatrixXd test_vel(6,1);
  test_vel << 0.05,0,0,0.01,0,0;

  VectorXd jointLimits(7);
  jointLimits << 170 - 5, 120 - 5, 170 - 5, 120 - 5, 170 - 5, 120 - 5, 175 - 5;
  jointLimits = jointLimits * 3.14159265358 / 180;
//   std::cout << "test_vel: " << test_vel <<std::endl;
//   test_vel = test_vel.transpose();

//   std::cout << "test_vel.T: " << test_vel << std::endl;
  
//   std::cout << "q: " << q << std::endl;

//   std::cout << "q_dot: " << q_dot << std::endl;


  Eigen::MatrixXd pseudo_inverse = Jt*(J*Jt).inverse();
  Eigen::MatrixXd joint_speeds = pseudo_inverse * (twist_desired- twist_actual);

//   std::cout << "Jacobian determinate: " << (Jt*(J*Jt)).determinant() << std::endl;

  
  auto denom = (q.array()*(1/jointLimits.array())).matrix().norm() * (4*jointLimits.array()*jointLimits.array());

  auto max_joint_limit_grad = (q.array() * (1/denom)).matrix() +.1*q_dot;
  

  // std::cout << "max joint limit grad: " << max_joint_limit_grad << std::endl;

  auto null_space = Eigen::MatrixXd::Identity(7,7) - pseudo_inverse*J;


  double null_space_grad_gain = 2;

  Eigen::DiagonalMatrix<double, 7> joint_gains(7);
  joint_gains.diagonal() << 50,50,50,50,100,100,50;
  
  commandedTorques =  joint_gains*H*(joint_speeds - (null_space_grad_gain*null_space*max_joint_limit_grad));

//   std::cout << "twist desired: " << twist_desired << std::endl;

//   commandedTorques =  Hi*Jt * (J * Hi * Jt).inverse() * (test_vel);



  for (int i = 0; i < 7; i++) {
    if (abs(q(i)) > jointLimits(i)) {
        std::cout << "joint limit exceeded on joint " << i+1 << std::endl;
        // commandedTorques(i,1) = 
        
        // std::cout << "quitting..." << std::endl;
        // exit(0);
    }
  }

  

  // Limit maximum commanded velocities
  for (int i = 0; i < num_joints_; i++) {
      if (commandedTorques(i, 0) > joint_torque_limit_) {
          commandedTorques(i, 0) = joint_torque_limit_;
          std::cout << "Warning: joint " << i << " commanded torque exceeded ";
          std::cout << "given limit of " << joint_torque_limit_ << std::endl;
      } else if (commandedTorques(i, 0) < -joint_torque_limit_) {
          commandedTorques(i, 0) = -joint_torque_limit_;
          std::cout << "Warning: joint " << i << " commanded torque exceeded ";
          std::cout << "given limit of " << -joint_torque_limit_ << std::endl;
      }
  }


//   std::cout << "commandedTorques: " << commandedTorques << std::endl;

  // Storing them in the output vector

  

  if(q.norm() == 0) {
    Eigen::VectorXd zero(7);
    zero << 0,0,0,0,0,0,0;
    commandedTorques = zero;

  }

  output->set_value(commandedTorques); // (7 x 6) * (6 x 1) = 7 x 1

}

} // namespace systems
} // namespace dairlib
