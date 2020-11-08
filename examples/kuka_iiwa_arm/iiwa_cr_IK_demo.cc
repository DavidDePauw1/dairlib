#include "drake/common/value.h"
#include "drake/common/trajectories/piecewise_polynomial.h"
#include "drake/common/trajectories/piecewise_quaternion.h"
#include "drake/manipulation/kuka_iiwa/iiwa_status_receiver.h"
#include "drake/manipulation/kuka_iiwa/iiwa_command_sender.h"
#include "drake/manipulation/planner/differential_inverse_kinematics_integrator.h"
#include "drake/manipulation/planner/differential_inverse_kinematics.h"
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/lcm/lcm_interface_system.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/diagram.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/primitives/trajectory_source.h"
#include "drake/systems/primitives/constant_vector_source.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/lcm/connect_lcm_scope.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/lcmt_iiwa_command.hpp"
#include "drake/lcmt_iiwa_status.hpp"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/systems/primitives/multiplexer.h"
#include "drake/systems/primitives/demultiplexer.h"
#include "drake/manipulation/planner/constraint_relaxing_ik.h"
#include "drake/math/rigid_transform.h"



#include "systems/controllers/endeffector_velocity_controller.h"
#include "systems/controllers/endeffector_position_controller.h"
#include <lcm/lcm-cpp.hpp>

#include "json.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>

#include <Eigen/Geometry>


using drake::manipulation::planner::ConstraintRelaxingIk;
using IKWaypoint = drake::manipulation::planner::ConstraintRelaxingIk::IkCartesianWaypoint;

using json = nlohmann::json;
namespace dairlib {

// CsvVector class: Takes CSV file as a parameter,
// loads data into a 2D vector of doubles, provides
// method to get the 2D vector.
class CsvVector {
  private:
    std::vector<std::vector<double>> data;
  public:
    CsvVector(std::string fileName) {
      //Opens stream to fileName.
      std::ifstream stream;
      stream.open(fileName);

      //Checks to make sure it has been opened.
      if (stream.is_open()) {
        std::cout << "CSV file " + fileName + " opened successfully" << std::endl;
      }

      //Finds number of columns in CSV file to resize data vector.
      std::string firstLine;
      std::getline(stream, firstLine);
      std::stringstream lineStream(firstLine);
      std::string dat;
      int length = 0;
      while (std::getline(lineStream, dat, ',')) {
        data.resize(data.size() + 1);
        data[length].push_back(std::stod(dat));
        length++;
      }

      //Loads CSV file contents into data vector.
      while(std::getline(stream, firstLine)) {
        int col = 0;
        std::stringstream moreLines(firstLine);
        while (std::getline(moreLines, dat, ',')) {
          data[col].push_back(std::stod(dat));
          col++;
        }
      }
      stream.close();
    }
    //Returns the data array.
    std::vector<std::vector<double>> getArray() {
      return data;
    }

    int knotSize() {
        return data[0].size();
    }
};

class InitialPosHandler {
    public:
        InitialPosHandler(){
            messageReceived = false;
        }

        ~InitialPosHandler() {}

        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                           const std::string& chan,
                           const drake::lcmt_iiwa_status* msg) {
            initialPositions << (double)msg->joint_position_measured[0],
                                (double)msg->joint_position_measured[1],
                                (double)msg->joint_position_measured[2],
                                (double)msg->joint_position_measured[3],
                                (double)msg->joint_position_measured[4],
                                (double)msg->joint_position_measured[5],
                                (double)msg->joint_position_measured[6];
            messageReceived = true;
        }

        bool messageReceived;
        Eigen::VectorXd initialPositions = Eigen::VectorXd(7);
};


class TransformMessageReciever : public drake::systems::LeafSystem<double> {
  public: 
    DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(TransformMessageReciever)
    std::string message;
    TransformMessageReciever() {
      // this->DeclareAbstractInputPort();
      // //could add in rotation component (probably useful in t
      
      // this->DeclareVectorOutputPort("translation output",&TransformMessageReciever::printValue);
      // this->DeclareVectorOutputPort("rotation output", )
      // this->DeclareAbstractOutputPort()
    }

    void translationOutput(const drake::systems::Context<double>& context, systems::BasicVector<double>* value) const 
    {


    }

    void rotationOutput(const drake::systems::Context<double>& context, systems::BasicVector<double>* value) const 
    {

    }

    void transformOutput(const drake::systems::Context<double>& context, drake::math::RigidTransform<double>* value) const {
      
    }
};


// This function creates a controller for a Kuka LBR Iiwa arm by connecting an
// EndEffectorPositionController to an EndEffectorVelocityController to control
// the individual joint torques as to move the endeffector
// to a desired position.

int do_main(int argc, char* argv[]) {
  

  std::ifstream settings_file("examples/kuka_iiwa_arm/simulation_settings.json");
  if (settings_file.is_open()) {
    std::cout << "Json file opened successfully." << std::endl;
  }
  //Initializes joint_gains json object
  json settings = json::parse(settings_file);

  //Kp and 'Rotational' Kp
  const std::string trajectory_file_path = settings["trajectory_file"];


  // Initialize Kuka model URDF-- from Drake kuka simulation files
  std::string kModelPath = "../drake/manipulation/models/iiwa_description"
                           "/iiwa7/iiwa7_no_collision.sdf";
  const std::string urdf_string = FindResourceOrThrow(kModelPath);
  
  // MultibodyPlants are created here, then passed by reference
  // to the controller blocks for internal modelling.
  const auto X_WI = drake::math::RigidTransform<double>::Identity();
  std::unique_ptr<MultibodyPlant<double>> owned_plant =
      std::make_unique<MultibodyPlant<double>>(0.0);



  drake::multibody::Parser plant_parser(owned_plant.get());
  
  const drake::multibody::ModelInstanceIndex iiwa_model = plant_parser.AddModelFromFile(urdf_string, "iiwa");
  owned_plant->WeldFrames(owned_plant->world_frame(),
                          owned_plant->GetFrameByName("iiwa_link_0"), X_WI);

    //add end effector to match plant in simulation
  const std::string ee_sdf =
        "examples/kuka_iiwa_arm/models/endeffector_attachment.sdf";
    const drake::multibody::ModelInstanceIndex ee_model =
        plant_parser.AddModelFromFile(ee_sdf, "ee_rod");
    owned_plant.get()->WeldFrames(
        owned_plant->GetFrameByName("iiwa_link_7", iiwa_model),
        owned_plant->GetFrameByName("ee_body", ee_model), X_WI);

  owned_plant->Finalize();
  
  drake::systems::DiagramBuilder<double> builder;

  auto lcm = builder.AddSystem<drake::systems::lcm::LcmInterfaceSystem>();

  // Adding status subscriber and receiver blocks
  auto status_subscriber = builder.AddSystem(
      drake::systems::lcm::LcmSubscriberSystem::Make<drake::lcmt_iiwa_status>(
          "IIWA_STATUS", lcm));
  auto status_receiver = builder.AddSystem<
      drake::manipulation::kuka_iiwa::IiwaStatusReceiver>();

  const std::string link_7 = "iiwa_link_7";

  //Processes Trajectories CSV file.
  CsvVector waypoints(trajectory_file_path);

  lcm::LCM lcm_;
  if (!lcm_.good()) return 1;

  Eigen::Vector3d x_initial;
  InitialPosHandler handler;
  lcm_.subscribe("IIWA_STATUS", &InitialPosHandler::handleMessage, &handler);
  while (lcm_.handle() == 0 && !handler.messageReceived);

  const std::unique_ptr<Context<double>> plant_context =
      owned_plant->CreateDefaultContext();
  owned_plant->SetPositions(plant_context.get(), handler.initialPositions);

  drake::math::RigidTransform<double> init_pos = owned_plant-> CalcRelativeTransform(*plant_context, owned_plant->world_frame(),owned_plant->GetFrameByName(link_7));
  Eigen::Vector3d checkpointTwo;
  checkpointTwo << waypoints.getArray()[1][0], waypoints.getArray()[2][0],
                   waypoints.getArray()[3][0];

  // 1/second m/s movement speed
  
  int firstMovementTime = 5;

  //Initializes trajectories to trajectoryVectors array.
  std::vector<drake::Quaternion<double>> orient_points;
  std::vector<Eigen::MatrixXd> trajectoryVectors;

  if (firstMovementTime > 0.0){
      trajectoryVectors.push_back(init_pos.translation());
      orient_points.push_back(init_pos.rotation().ToQuaternion());
  }

  for (unsigned int x = 0; x < waypoints.getArray()[0].size(); x++) {
    Eigen::Vector3d temp;
    temp << waypoints.getArray()[1][x], waypoints.getArray()[2][x],
            waypoints.getArray()[3][x];
    trajectoryVectors.push_back(temp);

    Eigen::Vector4d aPoint;
    aPoint << waypoints.getArray()[4][x], waypoints.getArray()[5][x],
              waypoints.getArray()[6][x], waypoints.getArray()[7][x];
    //using this constructor to preserve w,x,y,z ordering, see quaternion.h for details
    orient_points.push_back(drake::Quaternion<double>(aPoint(0),aPoint(1),aPoint(2),aPoint(3)));

  }

  std::vector<double> timeKnots = waypoints.getArray()[0];
  // Add the movement time to every timestep
  if (firstMovementTime > 0.0) {
      for (double& d : timeKnots) d += abs(firstMovementTime);
      std::cout << firstMovementTime << std::endl;
      timeKnots.insert(timeKnots.begin(), 0.0);
  }
  auto ee_trajectory =
      drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
          timeKnots, trajectoryVectors);
  auto ee_velocity = ee_trajectory.derivative(1);
  

  auto orientation_trajectory =
      drake::trajectories::PiecewiseQuaternionSlerp<double>(
          timeKnots, orient_points);

  for(int i = 0; i < orient_points.size(); i++){
    auto quat = orient_points[i];
    std::cout << "orient_points from read in: " << quat.coeffs() <<std::endl;
  }


  //use the target trajectory to create waypoints 
  ConstraintRelaxingIk inverse_kinematics(kModelPath,link_7);

  std::vector<IKWaypoint> IKWaypoints;
  std::vector<Eigen::VectorXd> q_sol;
  
  
  double time_step = 2;
  double cur_time = time_step;
  int time_ind = 1;
  
  std::vector<double> segmented_time;
  segmented_time.push_back(0);
  //fill out IKWaypoints with target trajectory
  while (time_ind < timeKnots.size())
  {
    double this_time;
    std::cout << "cur_time: " << cur_time<<std::endl;
    std::cout << "time_ind: " << time_ind << std::endl;
    std::cout << "timeKnot time: " << timeKnots[time_ind] << std::endl;
    if(cur_time == timeKnots[time_ind])
    {
      this_time = cur_time;
      cur_time+=time_step;
      time_ind++;
    }
    else if(cur_time > timeKnots[time_ind])
    {
      this_time = timeKnots[time_ind];
      cur_time = timeKnots[time_ind] +time_step;
      time_ind++;
      
    }
    else
    {
      this_time = cur_time;
      cur_time+=time_step;
    }
    std::cout << "this_time: " << this_time<< std::endl<<std::endl;  
    
    segmented_time.push_back(this_time);
    auto quat = orientation_trajectory.orientation(this_time);
    std::cout << "orientation quaternion: " <<quat.coeffs() << std::endl;
    drake::math::RigidTransform<double> wp_pose(quat,ee_trajectory.value(this_time));
    // drake::math::RigidTransform<double> wp_pose(drake::Quaternion<double>(0,0,1,0),ee_trajectory.value(this_time));

    //constrain orientation may not always need to be true
    IKWaypoint wp = {pose : wp_pose, pos_tol:{drake::Vector3<double>(0.01,0.01,0.01)}, rot_tol:0.05, constrain_orientation : true};
    IKWaypoints.push_back(wp);
    
  }
  auto start = std::chrono::system_clock::now();
  inverse_kinematics.PlanSequentialTrajectory(IKWaypoints, handler.initialPositions, &q_sol);
  auto end = std::chrono::system_clock::now();
  std::cout << "trajectory planning time: " << (end-start).count()<< std::endl;

  //q_sol can now be turned into a trajectory source for feed forward position control
  std::vector<Eigen::MatrixXd> joint_positions;

  std::cout << "segmented_time size: " << segmented_time.size() << std::endl;
  std::cout << "joint_positions size: " << q_sol.size() << std::endl;
  for(int i = 0; i < timeKnots.size(); i++){
    std::cout << "timeKnots: " << timeKnots[i] <<std::endl;
  }

  for(int i = 0; i < segmented_time.size(); i++){
    std::cout << "segemented_time: " << segmented_time[i] <<std::endl;
  }

  for(int i = 0; i < q_sol.size(); i++){
    std::cout << "joint_positions from crik: " << q_sol[i] <<std::endl;
  }
  
  for(auto j:q_sol) joint_positions.push_back(j);

  auto joint_trajectory =
      drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
          segmented_time, joint_positions);
          
  auto joint_trajectory_source = builder.AddSystem<drake::systems::TrajectorySource>(
        joint_trajectory);

  // Adding command publisher and broadcaster blocks
  auto command_sender = builder.AddSystem<
      drake::manipulation::kuka_iiwa::IiwaCommandSender>();
  auto command_publisher = builder.AddSystem(
      drake::systems::lcm::LcmPublisherSystem::Make<drake::lcmt_iiwa_command>(
          "IIWA_COMMAND", lcm, 1.0/200.0));


  builder.Connect(joint_trajectory_source->get_output_port(),
                  command_sender->get_position_input_port());
  builder.Connect(command_sender->get_output_port(),
                  command_publisher->get_input_port());

  auto diagram = builder.Build();


  drake::systems::Simulator<double> simulator(*diagram);


  simulator.set_publish_every_time_step(false);
  simulator.set_target_realtime_rate(1.0);
  simulator.Initialize();
  simulator.AdvanceTo(ee_trajectory.end_time());
  return 0;
}

} // Namespace dairlib

int main(int argc, char* argv[]) {
  return dairlib::do_main(argc, argv);
}
