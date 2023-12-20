//
// Created by qiayuan on 2022/7/24.
//

#include "legged_controllers/TargetTrajectoriesPublisher.h"

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

using namespace legged;

namespace {
scalar_t TARGET_DISPLACEMENT_VELOCITY;
scalar_t TARGET_ROTATION_VELOCITY;
scalar_t COM_HEIGHT;
vector_t DEFAULT_JOINT_STATE(12);
scalar_t TIME_TO_TARGET;
}  // namespace

scalar_t estimateTimeToTarget(const vector_t& desiredBaseDisplacement) {
  const scalar_t& dx = desiredBaseDisplacement(0);
  const scalar_t& dy = desiredBaseDisplacement(1);
  const scalar_t& dyaw = desiredBaseDisplacement(3);
  const scalar_t rotationTime = std::abs(dyaw) / TARGET_ROTATION_VELOCITY;
  const scalar_t displacement = std::sqrt(dx * dx + dy * dy);
  const scalar_t displacementTime = displacement / TARGET_DISPLACEMENT_VELOCITY;
  return std::max(rotationTime, displacementTime);
}

TargetTrajectories targetPoseToTargetTrajectories(const vector_t& targetPose, const SystemObservation& observation,
                                                  const scalar_t& targetReachingTime) {
  // desired time trajectory
  const scalar_array_t timeTrajectory{observation.time, targetReachingTime};

  // desired state trajectory
  vector_t currentPose = observation.state.segment<6>(6);
  currentPose(2) = COM_HEIGHT;
  currentPose(4) = 0;
  currentPose(5) = 0;
  vector_array_t stateTrajectory(2, vector_t::Zero(observation.state.size()));
  stateTrajectory[0] << vector_t::Zero(6), currentPose, DEFAULT_JOINT_STATE;
  stateTrajectory[1] << vector_t::Zero(6), targetPose, DEFAULT_JOINT_STATE;

  // desired input trajectory (just right dimensions, they are not used)
  const vector_array_t inputTrajectory(2, vector_t::Zero(observation.input.size()));

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

TargetTrajectories goalToTargetTrajectories(const vector_t& goal, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = goal(0);
    target(1) = goal(1);
    target(2) = COM_HEIGHT;
    target(3) = goal(3);
    target(4) = 0;
    target(5) = 0;
    return target;
  }();
  const scalar_t targetReachingTime = observation.time + estimateTimeToTarget(targetPose - currentPose);
  return targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
}

TargetTrajectories cmdVelToTargetTrajectories(const vector_t& cmdVel, const SystemObservation& observation) {
  const vector_t currentPose = observation.state.segment<6>(6);
  const Eigen::Matrix<scalar_t, 3, 1> zyx = currentPose.tail(3);
  vector_t cmdVelRot = getRotationMatrixFromZyxEulerAngles(zyx) * cmdVel.head(3);

  const scalar_t timeToTarget = TIME_TO_TARGET;
  const vector_t targetPose = [&]() {
    vector_t target(6);
    target(0) = currentPose(0) + cmdVelRot(0) * timeToTarget;
    target(1) = currentPose(1) + cmdVelRot(1) * timeToTarget;
    target(2) = COM_HEIGHT;
    target(3) = currentPose(3) + cmdVel(3) * timeToTarget;
    target(4) = 0;
    target(5) = 0;
    return target;
  }();

  // target reaching duration
  const scalar_t targetReachingTime = observation.time + timeToTarget;
  auto trajectories = targetPoseToTargetTrajectories(targetPose, observation, targetReachingTime);
  trajectories.stateTrajectory[0].head(3) = cmdVelRot;
  trajectories.stateTrajectory[1].head(3) = cmdVelRot;
  return trajectories;
}

TargetTrajectories gbplToTargetTrajectories(const quad_msgs::RobotPlan::ConstPtr& msg, const SystemObservation& observation) {
  scalar_array_t timeTrajectory(msg->plan_indices.size(), 0);
  vector_array_t stateTrajectory(msg->plan_indices.size(), vector_t::Zero(observation.state.size()));
  vector_array_t inputTrajectory(msg->plan_indices.size(), vector_t::Zero(observation.input.size()));

  std::cout << "current ob time: " << observation.time << std::endl;
  std::cout << "msg->states[0].header.stamp.toSec(): " << msg->states[0].header.stamp.toSec() << std::endl;
  std::cout << "rosTimenow.toSec(): " << ros::Time::now().toSec() << std::endl;

  scalar_t currentTime = observation.time;
  // scalar_t currentRosTime = msg->states[0].header.stamp.toSec();
  scalar_t currentRosTime = ros::Time::now().toSec();
  
  for(size_t i = 0; i < msg->plan_indices.size(); i++) {
    
    double time = msg->states[i].header.stamp.toSec() - currentRosTime + currentTime;
    timeTrajectory[i] = time;

    vector_t robotState = quad_utils::bodyStateMsgToEigen(msg->states[i].body); // [position rpy linearvel angularvel]
    // stateTrajectory[i].head(3) = robotState.segment<3>(6);
    // stateTrajectory[i].segment<3>(3) = robotState.tail(3);
    // stateTrajectory[i].segment<3>(6) = robotState.segment<3>(0);
    // stateTrajectory[i].segment<3>(9) = robotState.segment<3>(3).reverse();

    // Kmpc
    stateTrajectory[i].head(3) = robotState.head(3); //xyz
    stateTrajectory[i].segment<3>(3) = robotState.segment<3>(3).reverse(); //rpy
    inputTrajectory[i].head(3) = robotState.segment<3>(3); //linear vel
    inputTrajectory[i].segment<3>(3) = robotState.tail(3); //angular vel


    stateTrajectory[i].tail(12) = DEFAULT_JOINT_STATE;
    // Eigen::Quaterniond quat;
    // quat.x() = msg->states[i].body.pose.orientation.x;
    // quat.y() = msg->states[i].body.pose.orientation.y;
    // quat.z() = msg->states[i].body.pose.orientation.z;
    // quat.w() = msg->states[i].body.pose.orientation.w;
  }

  return {timeTrajectory, stateTrajectory, inputTrajectory};
}

int main(int argc, char** argv) {
  // const std::string robotName = "legged_robot";
  const std::string robotName = "mobile_manipulator";

  // Initialize ros node
  ::ros::init(argc, argv, robotName + "_target");
  ::ros::NodeHandle nodeHandle;
  // Get node parameters
  std::string referenceFile;
  std::string taskFile;
  nodeHandle.getParam("/referenceFile", referenceFile);
  nodeHandle.getParam("/taskFile", taskFile);

  loadData::loadCppDataType(referenceFile, "comHeight", COM_HEIGHT);
  loadData::loadEigenMatrix(referenceFile, "defaultJointState", DEFAULT_JOINT_STATE);
  loadData::loadCppDataType(referenceFile, "targetRotationVelocity", TARGET_ROTATION_VELOCITY);
  loadData::loadCppDataType(referenceFile, "targetDisplacementVelocity", TARGET_DISPLACEMENT_VELOCITY);
  loadData::loadCppDataType(taskFile, "mpc.timeHorizon", TIME_TO_TARGET);

  TargetTrajectoriesPublisher target_pose_command(nodeHandle, robotName, &goalToTargetTrajectories, &cmdVelToTargetTrajectories, &gbplToTargetTrajectories);

  ros::spin();
  // Successful exit
  return 0;
}
