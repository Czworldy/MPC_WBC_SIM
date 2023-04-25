// pinocchio
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>

// c++
#include <fstream>
#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/init.h>
#include <urdf_parser/urdf_parser.h>
// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>
// ocs2
#include <ocs2_core/misc/Display.h>
#include <ocs2_core/misc/LoadStdVectorOfPair.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <ocs2_self_collision/SelfCollisionConstraint.h>
#include <ocs2_self_collision/SelfCollisionConstraintCppAd.h>
#include <ocs2_msgs/mpc_observation.h>

#include "ocs2_jypro/LeggedRobotPreComputation.h"
// #include "ocs2_jypro/constraint/LeggedRobotWithArmSelfCollisionConstraint.h"
// #include "ocs2_jypro/constraint/JointPositionLimits.h"
// #include "ocs2_jypro/constraint/JointVelocityLimits.h"
// #include "ocs2_jypro/LeggedRobotWithArmPinocchioMapping.h"
#include "ocs2_jypro/common/ModelSettings.h"
#include "ocs2_jypro/synchronized_module/SwitchedModelReferenceManager.h"
#include "ocs2_jypro/command/utility.h"
#include "ocs2_jypro/common/Types.h"
#include "ocs2_jypro/BodyPositionEstimator/BodyPositionEstimator.h"

using namespace ocs2;
using namespace legged_robot;

namespace {
    ModelSettings modelSettings;
    std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr;
    std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr;
    CentroidalModelInfo centroidalModelInfo;
    std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlannerPtr;
    std::unique_ptr<LeggedRobotPreComputation> preComputationPtr;
    // std::unique_ptr<LeggedRobotWithArmSelfCollisionConstraint> selfCollisionConstraintPtr;
    // std::unique_ptr<JointPositionLimits> jointPositionLimitsPtr;
    // std::unique_ptr<JointVelocityLimits> jointVelocityLimitsPtr;
    vector_t jointPositionUpperBound, jointPositionLowerBound;
    vector_t jointVelocityUpperBound, jointVelocityLowerBound;
    QuaternionToRPY quaternionToRPY;
    TargetTrajectories targetTrajectory;
    bool verbose(true);
    bool ifTargetCome(false);
    
    std::ofstream in_selfCollision;
    std::ofstream in_jointPositionLimits;
    std::ofstream in_jointVelocityLimits;
    std::ofstream in_baseVelocity_x, in_baseVelocity_y, in_baseVelocity_z, in_baseVelocity_roll, in_baseVelocity_pitch, in_baseVelocity_yaw;
    std::ofstream in_gripperPosition, in_gripperOrientation;
    std::ofstream in_time_gripperRoll, in_time_gripperPitch, in_time_gripperYaw;

    // #define numOfActuatedJoint 18
    // #define dofOfRobot 24
    // #define numOfArmDof 6
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, Eigen::Quaternion<scalar_t>> interpolateEndEffectorPose(scalar_t time) {

  const auto& eePositionTrajectory = targetTrajectory.eePositionTrajectory;

  vector_t position;
  Eigen::Quaternion<scalar_t> orientation;

  if (eePositionTrajectory.size() > 1) {
    // Normal interpolation case
    int index;
    scalar_t alpha;
    std::tie(index, alpha) = LinearInterpolation::timeSegment(time, targetTrajectory.timeTrajectory);

    const auto& lhs = eePositionTrajectory[index];
    const auto& rhs = eePositionTrajectory[index + 1];
    const Eigen::Quaternion<scalar_t> q_lhs(lhs.tail<4>());
    const Eigen::Quaternion<scalar_t> q_rhs(rhs.tail<4>());

    position = alpha * lhs.head<3>() + (1.0 - alpha) * rhs.head<3>();
    orientation = q_lhs.slerp((1.0 - alpha), q_rhs);
  } else {  // eePositionTrajectory.size() == 1
    position = eePositionTrajectory.front().head<3>();
    orientation = Eigen::Quaternion<scalar_t>(eePositionTrajectory.front().tail<4>());

    // std::cout << "[EndEffectorConstraint::interpolateEndEffectorPose]eePositionTrajectory.front() \n" << eePositionTrajectory.front() << std::endl;
  }
  return {position, orientation};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void leggedObservationCallback(const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    SystemObservation observation(ros_msg_conversions::readObservationMsg(*msg));
    // SelfConllision Value
    PinocchioInterface& pinocchioInterfacePreComp = preComputationPtr->getPinocchioInterface();
    const auto& model = pinocchioInterfacePreComp.getModel();
    auto& data = pinocchioInterfacePreComp.getData();
    const auto q = observation.state.tail(centroidalModelInfo.stateDim - 6);
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacements(model, data);
    pinocchio::computeJointJacobians(model, data);
    pinocchio::updateGlobalPlacements(model, data);
    // vector_t selfCollisionValue(selfCollisionConstraintPtr->getValue(observation.time, observation.state, *preComputationPtr));
    in_selfCollision << observation.time << "\t" << selfCollisionValue.minCoeff() << "\n";
    std::cerr << "[dqwang: selfCollisionValue] min: \n" << selfCollisionValue << "\n";

    // JointPositionLimits
    // vector_t jointPositionLimitsValue(jointPositionLimitsPtr->getValue(observation.time, observation.state, *preComputationPtr));
    // in_jointPositionLimits << observation.time << "\t" << (jointPositionLimitsValue - jointPositionLowerBound).minCoeff() 
    //                                            << "\t" << (jointPositionUpperBound - jointPositionLimitsValue).minCoeff() << "\n";

    // // JointVelocityLimits
    // vector_t jointVelocityLimitsValue(jointVelocityLimitsPtr->getValue(observation.time, observation.state, observation.input, *preComputationPtr));
    // in_jointVelocityLimits << observation.time << "\t" << (jointVelocityLimitsValue- jointVelocityLowerBound).minCoeff()
    //                                            << "\t" << (jointVelocityUpperBound - jointVelocityLimitsValue).minCoeff() << "\n";

    // BaseVelocity
    matrix_t InverseAb, Aj;
    vector6_t baseVelocity;
    const auto& Ag = pinocchio::computeCentroidalMap(model, data, q);
    pseudoInverse(Ag.leftCols(6), 0.0001, InverseAb);
    Aj = Ag.rightCols(numOfActuatedJoint);
    pinocchio::computeTotalMass(model, data);
    baseVelocity = InverseAb * (observation.state.head(6) * data.mass[0] - Aj * observation.input.tail(numOfActuatedJoint));
    in_baseVelocity_x << observation.time << "\t" << baseVelocity[0] << "\n";
    in_baseVelocity_y << observation.time << "\t" << baseVelocity[1] << "\n";
    in_baseVelocity_z << observation.time << "\t" << baseVelocity[2] << "\n";
    in_baseVelocity_roll  << observation.time << "\t" << baseVelocity[5] << "\n";
    in_baseVelocity_pitch << observation.time << "\t" << baseVelocity[4] << "\n";
    in_baseVelocity_yaw   << observation.time << "\t" << baseVelocity[3] << "\n";

    // GripperPosition
    const auto eeIndex = model.getBodyId(modelSettings.contactNames3DoF[4]);
    const vector_t eeCurrentPosition = data.oMf[eeIndex].translation();
    const quaternion_t eeCurrentOrientation = matrixToQuaternion(data.oMf[eeIndex].rotation());
    const vector_t eeCurrentRPY = quaternionToRPY.quaternionToTotalRad(eeCurrentOrientation);
    in_gripperPosition << eeCurrentPosition[0] << "\t" << eeCurrentPosition[1] << "\t" << eeCurrentPosition[2] << "\n";
    in_gripperOrientation << eeCurrentRPY[0] << "\t" << eeCurrentRPY[1] << "\t" << eeCurrentRPY[2] << "\n";

    // TargetTrajectory
    if (ifTargetCome) {
        const auto desiredPositionOrientation = interpolateEndEffectorPose(observation.time);
        const auto desiredRPY = quaternionToRPY.quaternionToTotalRad(desiredPositionOrientation.second);
        
        in_time_gripperRoll  << observation.time << "\t" << eeCurrentRPY[0] << "\t" << desiredRPY[0] << "\n";
        in_time_gripperPitch << observation.time << "\t" << eeCurrentRPY[1] << "\t" << desiredRPY[1] << "\n";
        in_time_gripperYaw   << observation.time << "\t" << eeCurrentRPY[2] << "\t" << desiredRPY[2] << "\n";
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void targetTrajectoryCallback(const ocs2_msgs::mpc_target_trajectories::ConstPtr& msg) {

    targetTrajectory = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
    ifTargetCome = true;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& taskFile) {
    const auto initModeSchedule = loadModeSchedule(taskFile, "initialModeSchedule", false);
    const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(taskFile, "defaultModeSequenceTemplate", false);

    const auto defaultGait = [&] {
        Gait gait{};
        gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
        // Events: from time -> phase
        std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                      [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
        // Modes:
        gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
        return gait;
    }();

    // display
    std::cerr << "\nInitial Modes Schedule: \n" << initModeSchedule << std::endl;
    std::cerr << "\nDefault Modes Sequence Template: \n" << defaultModeSequenceTemplate << std::endl;

    return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int main (int argc, char** argv) {
    const std::string robotName = "legged_robot";

    // Initialize ros node
    ros::init(argc, argv, robotName + "_DateRecorder");
    ros::NodeHandle nodeHandle;
    ros::Subscriber leggedObservationSubscriber, targetTrajectorySubscriber;
    leggedObservationSubscriber = nodeHandle.subscribe("/legged_robot_mpc_observation", 1, &leggedObservationCallback);
    targetTrajectorySubscriber = nodeHandle.subscribe("/legged_robot_mpc_target", 1, &targetTrajectoryCallback);
    // Get Parameters From ROS
    std::string taskFile, urdfFile, referenceFile;
    nodeHandle.getParam("/taskFile", taskFile);
    nodeHandle.getParam("/urdfFile", urdfFile);
    nodeHandle.getParam("/referenceFile", referenceFile);
    // ModelSettings
    modelSettings = loadModelSettings(taskFile, "model_settings", verbose);
    // PinocchioInterface
    pinocchioInterfacePtr.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings.jointNames)));
    // CentroidalModelInfo
    centroidalModelInfo = centroidal_model::createCentroidalModelInfo(*pinocchioInterfacePtr, centroidal_model::loadCentroidalType(taskFile),
                                centroidal_model::loadDefaultJointState(18, referenceFile), modelSettings.contactNames3DoF, modelSettings.contactNames6DoF);
    // SwingTrajectoryPlanner
    swingTrajectoryPlannerPtr.reset(new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config"), 4));
    // ReferenceManager
    referenceManagerPtr = std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(taskFile), std::move(swingTrajectoryPlannerPtr));
    // PreComputation
    preComputationPtr.reset(new LeggedRobotPreComputation(*pinocchioInterfacePtr, centroidalModelInfo, *referenceManagerPtr->getSwingTrajectoryPlanner(), modelSettings));
    // SelfCollisionConstraint
    // std::vector<std::pair<size_t, size_t>> collisionObjectPairs;
    // std::vector<std::pair<std::string, std::string>> collisionLinkPairs;
    // scalar_t minimumDistance = 0.0;

    // boost::property_tree::ptree pt;
    // boost::property_tree::read_info(taskFile, pt);
    // std::cerr << "\n #### SelfCollision Settings: ";
    // std::cerr << "\n #### ==========================================================\n";
    // loadData::loadPtreeValue(pt, minimumDistance, "selfCollision.minimumDistance", true);
    // loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionObjectPairs", collisionObjectPairs, true);
    // loadData::loadStdVectorOfPair(taskFile, "selfCollision.collisionLinkPairs", collisionLinkPairs, true);
    // std::cerr << "\n #### ==========================================================\n";

    // PinocchioGeometryInterface geometryInterface(*pinocchioInterfacePtr, collisionLinkPairs, collisionObjectPairs);
    // const size_t numCollisonPairs = geometryInterface.getNumCollisionPairs();
    // std::cerr << "SelfCollsion: Test for " << numCollisonPairs << " collision pairs\n";

    // selfCollisionConstraintPtr.reset(new LeggedRobotWithArmSelfCollisionConstraint(
    //                                     LeggedRobotWithArmPinocchioMapping(centroidalModelInfo), std::move(geometryInterface), minimumDistance));

    // JointPositionLimits
    // const int jointPositionLimitDim(9);
    // jointPositionLowerBound = vector_t::Zero(jointPositionLimitDim);
    // jointPositionUpperBound = vector_t::Zero(jointPositionLimitDim);
    // loadData::loadEigenMatrix(taskFile, "jointPositionLimits.lowerBound", jointPositionLowerBound);
    // loadData::loadEigenMatrix(taskFile, "jointPositionLimits.upperBound", jointPositionUpperBound);
    // std::cerr << "\n #### JointPositionLimits Settings: ";
    // std::cerr << "\n #### =============================================================================\n";
    // std::cerr << " #### lowerBound: " << jointPositionLowerBound.transpose() << '\n';
    // std::cerr << " #### upperBound: " << jointPositionUpperBound.transpose() << '\n';
    // jointPositionLimitsPtr.reset(new JointPositionLimits(jointPositionLimitDim));

    // // JointVelocityLimits
    // const int jointVelocityLimitDim(12);
    // jointVelocityLowerBound = vector_t::Zero(jointVelocityLimitDim);
    // jointVelocityUpperBound = vector_t::Zero(jointVelocityLimitDim);
    // loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.lowerBound", jointVelocityLowerBound);
    // loadData::loadEigenMatrix(taskFile, "jointVelocityLimits.upperBound", jointVelocityUpperBound);
    // std::cerr << "\n #### JointVelocityLimits Settings: ";
    // std::cerr << "\n #### =============================================================================\n";
    // std::cerr << " #### 'lowerBound':  " << jointVelocityLowerBound.transpose() << std::endl;
    // std::cerr << " #### 'upperBound':  " << jointVelocityUpperBound.transpose() << std::endl;
    // jointVelocityLimitsPtr.reset(new JointVelocityLimits(jointVelocityLimitDim));

    // Data Recorder
    in_selfCollision.open("/home/dqwang/data_exp/20220912/data/DMPC/selfCollision.txt", std::ios::trunc);
    in_jointPositionLimits.open("/home/dqwang/data_exp/20220912/data/DMPC/jointPositionLimits.txt", std::ios::trunc);
    in_jointVelocityLimits.open("/home/dqwang/data_exp/20220912/data/DMPC/jointVelocityLimits.txt", std::ios::trunc);
    in_baseVelocity_x.open("/home/dqwang/data_exp/20220912/data/DMPC/baseVelocity_x.txt", std::ios::trunc);
    in_baseVelocity_y.open("/home/dqwang/data_exp/20220912/data/DMPC/baseVelocity_y.txt", std::ios::trunc);
    in_baseVelocity_z.open("/home/dqwang/data_exp/20220912/data/DMPC/baseVelocity_z.txt", std::ios::trunc);
    in_baseVelocity_roll.open("/home/dqwang/data_exp/20220912/data/DMPC/baseVelocity_roll.txt", std::ios::trunc);
    in_baseVelocity_pitch.open("/home/dqwang/data_exp/20220912/data/DMPC/baseVelocity_pitch.txt", std::ios::trunc);
    in_baseVelocity_yaw.open("/home/dqwang/data_exp/20220912/data/DMPC/baseVelocity_yaw.txt", std::ios::trunc);
    in_gripperPosition.open("/home/dqwang/data_exp/20220912/data/DMPC/gripperPosition.txt", std::ios::trunc);
    in_gripperOrientation.open("/home/dqwang/data_exp/20220912/data/DMPC/gripperOrientation.txt", std::ios::trunc);
    in_time_gripperRoll.open("/home/dqwang/data_exp/20220912/data/DMPC/time_gripperRoll.txt", std::ios::trunc);
    in_time_gripperPitch.open("/home/dqwang/data_exp/20220912/data/DMPC/time_gripperPitch.txt", std::ios::trunc);
    in_time_gripperYaw.open("/home/dqwang/data_exp/20220912/data/DMPC/time_gripperYaw.txt", std::ios::trunc);

    // spin
    while (ros::ok() && ros::master::check()) {
        ros::spinOnce();
    }

    return 0;
}