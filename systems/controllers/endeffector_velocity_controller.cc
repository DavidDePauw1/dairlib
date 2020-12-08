#include "systems/controllers/endeffector_velocity_controller.h"

namespace dairlib{
namespace systems{

EndEffectorVelocityController::EndEffectorVelocityController(
        const MultibodyPlant<double>& plant, std::string ee_frame_name,
        Eigen::Vector3d ee_contact_frame, double k_d, double k_r,
        double joint_torque_limit,
        Eigen::DiagonalMatrix<double, 7> joint_torque_mult,
        double null_space_grad_gain,
        double joint_lim_grad_d ) : _plant(plant),
        _num_joints(_plant.num_positions()),
        _ee_joint_frame(_plant.GetFrameByName(ee_frame_name)),
        _joint_torque_mult(joint_torque_mult),
        _null_space_grad_gain(null_space_grad_gain),
        _joint_lim_grad_d(joint_lim_grad_d){

    // Set up this block's input and output ports
    // Input port values will be accessed via EvalVectorInput() later
    _joint_position_measured_port = this->DeclareVectorInputPort(
        "joint_position_measured", BasicVector<double>(_num_joints)).get_index();
    _joint_velocity_measured_port = this->DeclareVectorInputPort(
        "joint_velocity_measured", BasicVector<double>(_num_joints)).get_index();
    _endpoint_twist_commanded_port = this->DeclareVectorInputPort(
        "endpoint_twist_commanded", BasicVector<double>(6)).get_index();

    // Note that this function contains a pointer to the callback function below.
    _endpoint_torque_output_port = this->DeclareVectorOutputPort(
        BasicVector<double>(_num_joints),
        &EndEffectorVelocityController::CalcOutputTorques).get_index();

    failure_output_port_ = this->DeclareAbstractOutputPort(&EndEffectorVelocityController::failureCalc).get_index();


    _ee_contact_frame = ee_contact_frame;
    _k_d = k_d;
    _k_r = k_r;
    _joint_torque_limit = joint_torque_limit;
}

void EndEffectorVelocityController::failureCalc(const Context<double> &context, bool* failure) const{
    *failure = is_failing;
    
}

// Callback for DeclareVectorInputPort. No return value.
// The parameter 'output' is the output.
// This function is called many times a second.
void EndEffectorVelocityController::CalcOutputTorques(
  const Context<double>& context, BasicVector<double>* output) const {
  // We read the above input ports with EvalVectorInput
  // The purpose of CopyToVector().head(NUM_JOINTS) is to remove the timestamp
  // from the vector input ports

  is_failing = false;

  const BasicVector<double>* q_timestamped =
      (BasicVector<double>*) this->EvalVectorInput(
          context, _joint_position_measured_port);
  auto q = q_timestamped->get_value();

  const BasicVector<double>* q_dot_timestamped =
      (BasicVector<double>*) this->EvalVectorInput(
          context, _joint_velocity_measured_port);
  auto q_dot = q_dot_timestamped->get_value();

  const BasicVector<double>* twist_desired_timestamped =
      (BasicVector<double>*) this->EvalVectorInput(
          context, _endpoint_twist_commanded_port);
  auto twist_desired = twist_desired_timestamped->get_value();

  const std::unique_ptr<Context<double>> plant_context =
      _plant.CreateDefaultContext();
  _plant.SetPositions(plant_context.get(), q);
  _plant.SetVelocities(plant_context.get(), q_dot);



  // Calculating the jacobian of the kuka arm
  Eigen::MatrixXd J(6, _num_joints);
  _plant.CalcJacobianSpatialVelocity(
      *plant_context,drake::multibody::JacobianWrtVariable::kV, _ee_joint_frame, _ee_contact_frame, _plant.world_frame(), _plant.world_frame(),
      &J);
  Eigen::MatrixXd Jt = J.transpose();

  // Using the jacobian, calculating the actual current velocities of the arm
  MatrixXd twist_actual = J * q_dot;


  // Calculating the error
  Eigen::DiagonalMatrix<double, 6> gains(6);
  gains.diagonal() << _k_r, _k_r, _k_r, _k_d, _k_d, _k_d;

  MatrixXd error = gains * (twist_desired - twist_actual);

  VectorXd commandedTorques(_num_joints);


  // Calculating Mass Matrix
  Eigen::MatrixXd H(_num_joints, _num_joints);
  _plant.CalcMassMatrixViaInverseDynamics(*plant_context.get(), &H);

//------------testing-------------

  VectorXd jointLimits(7);
  jointLimits << 170 - 5, 120 - 5, 170 - 5, 120 - 5, 170 - 5, 120 - 5, 175 - 5;
  jointLimits = jointLimits * 3.14159265358 / 180;


  Eigen::MatrixXd pseudo_inverse = Jt*(J*Jt).inverse();

  double det = (J*Jt).determinant();
//   cause failure if determinant is too small
  if(det < .00001){
      is_failing = true;
      std::cout << "singularity based failure" << std::endl;
  }


  Eigen::MatrixXd joint_speeds = pseudo_inverse * (error);

  
  auto denom = (q.array()*(1/jointLimits.array())).matrix().norm() * (4*jointLimits.array()*jointLimits.array());

  auto max_joint_limit_grad = (q.array() * (1/denom)).matrix() + _joint_lim_grad_d*q_dot;

  auto null_space = Eigen::MatrixXd::Identity(7,7) - pseudo_inverse*J;

  commandedTorques =  _joint_torque_mult*H*(joint_speeds - (_null_space_grad_gain*null_space*max_joint_limit_grad));



  for (int i = 0; i < 7; i++) {
    if (abs(q(i)) > jointLimits(i)) {
        std::cout << "joint limit exceeded on joint " << i+1 << std::endl;
        // commandedTorques(i,0) = 0;


        is_failing = true;
        // std::cout << "quitting..." << std::endl;
        // exit(0);
    }
  }

  
  
  // Limit maximum commanded velocities and cause failure if exceeded
  for (int i = 0; i < _num_joints; i++) {
      if (commandedTorques(i, 0) > _joint_torque_limit) {
          commandedTorques(i, 0) = _joint_torque_limit;
          std::cout << "Warning: joint " << i << " commanded torque exceeded ";
          std::cout << "given limit of " << _joint_torque_limit << std::endl;

          std::cout << "joint limit based failure" << std::endl;

          is_failing = true;
      } else if (commandedTorques(i, 0) < -_joint_torque_limit) {
          commandedTorques(i, 0) = -_joint_torque_limit;
          std::cout << "Warning: joint " << i << " commanded torque exceeded ";
          std::cout << "given limit of " << -_joint_torque_limit << std::endl;
          is_failing = true;

          std::cout << "joint limit based failure" << std::endl;


      }
  }

  

  if(q.norm() == 0) {
    Eigen::VectorXd zero(7);
    zero << 0,0,0,0,0,0,0;
    commandedTorques = zero;

  }

  output->set_value(commandedTorques); // (7 x 6) * (6 x 1) = 7 x 1

}

} // namespace systems
} // namespace dairlib
