#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include "common/ModelSettings.h"
#include "common/ManipulatorModelInfo.h"
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>

#include <ocs2_ros_interfaces/command/TargetTrajectoriesUpperLayer.h>

using namespace ocs2;
using namespace legged_robot;

namespace {
scalar_t comHeight;
vector_t defaultJointState(18);
vector_t defaultLegJointState(12);
std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
ModelSettings modelSettings;
}  // namespace

TargetTrajectories commandPolicyToTargetTrajectories(const TargetTrajectories& commandPolicy, const SystemObservation& observation) {
  size_t N_times(commandPolicy.timeTrajectory.size());
  size_t stateDim(observation.state.size());
  size_t inputDim(observation.input.size());
  scalar_array_t timeTrajectory(N_times);
  vector_array_t stateTrajectory(N_times, vector_t::Zero(stateDim));
  vector_array_t inputTrajectory(N_times, vector_t::Zero(inputDim));
  vector6_t vBase;

  const auto& model = pinocchioInterfacePtr->getModel();
  auto& data = pinocchioInterfacePtr->getData();
  ManipulatorModelType manipulatorType = static_cast<ManipulatorModelType>(3);

  switch (manipulatorType) {
    case ManipulatorModelType::WheelBasedMobileManipulator: {
      for (int i(0); i<N_times; i++) {
        // copy time
        timeTrajectory[i] = commandPolicy.timeTrajectory[i] - commandPolicy.timeTrajectory[0] + observation.time;
        // desired state
        stateTrajectory[i] << vector_t::Zero(6), commandPolicy.stateTrajectory[i][0], commandPolicy.stateTrajectory[i][1], comHeight, commandPolicy.stateTrajectory[i][2], 0, 0, defaultLegJointState, commandPolicy.stateTrajectory[i].tail(6);
        const auto& Ag = pinocchio::computeCentroidalMap(model, data, stateTrajectory[i].tail(stateDim - 6)); 
        const auto theta = commandPolicy.stateTrajectory[i][2];
        const auto v = commandPolicy.inputTrajectory[i][0];
        vBase << cos(theta) * v, sin(theta) * v, 0, commandPolicy.inputTrajectory[i][1], 0, 0;
        // stateTrajectory[i].head(6) = Ag.leftCols(6) * vBase / data.mass[0];

        std::cout << "[hBase]: \n" << stateTrajectory[i].head(6) << std::endl;
        // desired input
        inputTrajectory[i] << vector_t::Zero(inputDim);
      }
      break;
    }
    case ManipulatorModelType::OmniBasedMobileManipulator: {
      for (int i(0); i<N_times; i++) {
        // copy time
        timeTrajectory[i] = commandPolicy.timeTrajectory[i] - commandPolicy.timeTrajectory[0] + observation.time;
        // desired state
        stateTrajectory[i] << vector_t::Zero(6), commandPolicy.stateTrajectory[i][0], commandPolicy.stateTrajectory[i][1], comHeight, commandPolicy.stateTrajectory[i][2], 0, 0, defaultLegJointState, commandPolicy.stateTrajectory[i].tail(6);
        // desired input
        inputTrajectory[i] << vector_t::Zero(inputDim);
      }    
      break;
    }
    case ManipulatorModelType::FullyActuatedFloatingArmManipulator: {
      for (int i(0); i<N_times; i++) {
        // copy time
        timeTrajectory[i] = commandPolicy.timeTrajectory[i] - commandPolicy.timeTrajectory[0] + observation.time;
        // desired state
        stateTrajectory[i] << vector_t::Zero(6), commandPolicy.stateTrajectory[i].head(6), defaultLegJointState, commandPolicy.stateTrajectory[i].tail(6);
        stateTrajectory[i][8] = commandPolicy.stateTrajectory[i][2] + comHeight;
        // desired input
        inputTrajectory[i] << vector_t::Zero(inputDim);
      }    
      break;
    }
  }

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char* argv[]) {
  // ros node handle
  const std::string robotName = "legged_robot";

  ::ros::init(argc, argv, robotName + "_target_");
  ::ros::NodeHandle nodeHandle;
  std::string targetCommandFile;
  nodeHandle.getParam("/referenceFile", targetCommandFile);
  const std::string upperName("/mobile_manipulator");

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(targetCommandFile, pt);
  comHeight = pt.get<scalar_t>("comHeight");
  ocs2::loadData::loadEigenMatrix(targetCommandFile, "defaultJointState", defaultJointState);
  defaultLegJointState = defaultJointState.head(12);

  // URDF Model -> Pinocchio Model
  std::string urdfPath = "/home/yjy/MPC_WBC_sim/ocs2_ws/src/X20_ARM/urdf/X20_ARM_ocs2.urdf";
  pinocchioInterfacePtr.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfPath, modelSettings.jointNames)));

  // // ros node handle
  // ::ros::init(argc, argv, robotName + "_target");
  // ::ros::NodeHandle nodeHandle;

  // Publish Command Policy From Upper Layer
  TargetTrajectoriesUpperLayer targetUpperCommand(nodeHandle, robotName, upperName, &commandPolicyToTargetTrajectories);
  targetUpperCommand.publishCommandPolicyFromUpperLayer();

}