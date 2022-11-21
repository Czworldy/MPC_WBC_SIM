#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include <LeggedRobotWithArmVisualizer.h>
#include "LeggedRobotWithArmInterface.h"

#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <urdf_parser/urdf_parser.h>

using namespace ocs2;
using namespace legged_robot;

int main(int argc, char** argv) {
    const std::string robotName = "legged_robot";

    // Initialize ros node
    ros::init(argc, argv, robotName + "_mrt");
    ros::NodeHandle nodeHandle;
    // Get node parameters
    std::string taskFile, urdfFile, referenceFile;
    nodeHandle.getParam("/taskFile", taskFile);
    nodeHandle.getParam("/referenceFile", referenceFile);
    nodeHandle.getParam("/urdfFile", urdfFile);

    LeggedRobotWithArmInterface interface(taskFile, urdfFile, referenceFile);

    // MRT
    MRT_ROS_Interface mrt(robotName);
    mrt.initRollout(&interface.getRollout());
    mrt.launchNodes(nodeHandle);

    // Visualization
    CentroidalModelPinocchioMapping pinocchioMapping(interface.getCentroidalModelInfo());
    PinocchioEndEffectorKinematics endEffectorKinematics(interface.getPinocchioInterface(), pinocchioMapping,
                                                         interface.modelSettings().contactNames3DoF);
    std::shared_ptr<LeggedRobotWithArmVisualizer> leggedRobotWithArmVisualizer(
            new LeggedRobotWithArmVisualizer(interface.getPinocchioInterface(), interface.getCentroidalModelInfo(), endEffectorKinematics, nodeHandle));
    
    // Dummy legged robot
    MRT_ROS_Dummy_Loop leggedRobotWithArmDummySimulator(mrt, interface.mpcSettings().mrtDesiredFrequency_, 
                                                        interface.mpcSettings().mpcDesiredFrequency_);
    leggedRobotWithArmDummySimulator.subscribeObservers({leggedRobotWithArmVisualizer});

    // Initial state
    SystemObservation initObservation;
    initObservation.state = interface.getInitialState();
    initObservation.input = vector_t::Zero(interface.getCentroidalModelInfo().inputDim);
    initObservation.mode = ModeNumber::STANCE;


    // Initial eePosition
    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
    pinocchioInterfacePtr_.reset(new PinocchioInterface(ocs2::centroidal_model::createPinocchioInterface(urdfFile, interface.modelSettings().jointNames)));
    const auto& model = pinocchioInterfacePtr_->getModel();
    auto& data = pinocchioInterfacePtr_->getData(); 
    const auto& q = initObservation.state.tail(initObservation.state.rows() - 6);
    
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);

    vector_t eePosition = data.oMf[pinocchioInterfacePtr_->getModel().getBodyId(interface.modelSettings().contactNames3DoF[4])].translation();


    // Initial command
    TargetTrajectories initTargetTrajectories({0.0}, {initObservation.state}, {initObservation.input}, {eePosition});

    // Run dummy
    leggedRobotWithArmDummySimulator.run(initObservation, initTargetTrajectories);

    // Successful exit
    return 0;
}