#include <ros/init.h>
#include <urdf_parser/urdf_parser.h>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>

#include "LeggedRobotWithArmInterface.h"
#include "GaitReceiver.h"

#include <ocs2_sqp/MultipleShootingMpc.h>

int main(int argc, char** argv) {
    const std::string robotName = "legged_robot";

    // Initialize ros node
    ros::init(argc, argv, robotName + "_mpc");
    ros::NodeHandle nodeHandle;

    std::string taskFile, urdfFile, referenceFile;
    nodeHandle.getParam("/taskFile", taskFile);
    nodeHandle.getParam("/referenceFile", referenceFile);
    nodeHandle.getParam("/urdfFile", urdfFile);


    // Robot interface
    ocs2::legged_robot::LeggedRobotWithArmInterface interface(taskFile, urdfFile, referenceFile);

    // Gait receiver
    auto gaitReceiverPtr = std::make_shared<ocs2::legged_robot::GaitReceiver>(
            nodeHandle, interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);
    
    // ROS ReferenceManager
    auto rosReferenceManagerPtr = std::make_shared<ocs2::RosReferenceManager>(robotName, interface.getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(nodeHandle);

    // MPC
    // ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(), interface.getOptimalControlProblem(),
    //                              interface.getInitializer());
    ocs2::MultipleShootingMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(),
                                  interface.getInitializer());
    mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);

    // Launch MPC ROS node
    ocs2::MPC_ROS_Interface mpcNode(mpc, robotName);
    mpcNode.launchNodes(nodeHandle);

    // Successful exit
    return 0;
}