/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include "ocs2_jypro/command/TargetTrajectoriesPublisher.h"

#include "ocs2_jypro/common/Types.h"
#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/Display.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_msgs/mpc_target_trajectories.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>

// #include <Eigen/Core>
// #include <Eigen/Dense>
// #include <Eigen/Geometry>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesPublisher::TargetTrajectoriesPublisher(::ros::NodeHandle &nodeHandle, const std::string &topicPrefix,
                                   const vector_t &defaultJointState, PinocchioInterface& pinocchioInterface,
                                   const CentroidalModelInfo& centroidalModelInfo)
    : defaultJointState_(defaultJointState),
      pinocchioInterface_(pinocchioInterface),
      centroidalModelInfo_(centroidalModelInfo) {
    // Mapping
    mappingPtr_ = std::make_shared<ocs2::CentroidalModelPinocchioMapping>(centroidalModelInfo_);
    mappingPtr_->setPinocchioInterface(pinocchioInterface_);
    // observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(latestObservationMutex_);
        latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
        this->isMpcPolicyCome = true;
    };
    observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);

    auto trajCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(latestJoyMsgsMutex_);
        receivedTargetTrajectories_ = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);
        this->isJoyMsgsCome = true;
    };
    joySubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_target_trajectories>("/traj", 1, trajCallback);
    // auto joyCallback = [this](const geometry_msgs::Twist::ConstPtr &msg) {
    //     std::lock_guard<std::mutex> lock(latestJoyMsgsMutex_);

    //     deltaX = msg->linear.x;
    //     deltaY = msg->linear.y; // msg->linear.x
    //     deltaYaw = msg->angular.z;
    //     this->isJoyMsgsCome = true;
    // };

    // joySubscriber_ = nodeHandle.subscribe<geometry_msgs::Twist>("/vel", 1, joyCallback);

    // Trajectories publisher
    targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, topicPrefix));
    // for Visualization
    TargetTrajectoriesVisualizerPublisher_ = nodeHandle.advertise<geometry_msgs::PoseArray>("legged_robot_targetvisualizer", 2);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesPublisher::publishKeyboardCommand(const std::string &commadMsg) {
    ::ros::Rate rate(1);
    while (ros::ok() && ros::master::check()) {
        ::ros::spinOnce();
        if (isJoyMsgsCome && isMpcPolicyCome) {
            std::lock_guard<std::mutex> lock(latestJoyMsgsMutex_);

            std::cout << "The following command is received: [\n" << receivedTargetTrajectories_ << "]\n\n";

            SystemObservation observation;
            {
                std::lock_guard<std::mutex> lock(latestObservationMutex_);
                observation = latestObservation_;
            }
            const auto targetTrajectories = getTargetTrajectories(observation);

            // publish TargetTrajectories
            targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
            this->isJoyMsgsCome = false;
            // for visualization
            // create pose_array (along trajectory)
            geometry_msgs::PoseArray teb_poses;
            teb_poses.header.frame_id = "odom";
            teb_poses.header.stamp = ros::Time::now();
            
            // fill path msgs with teb configurations
            for (int i=0; i < targetTrajectories.timeTrajectory.size(); i++)
            {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "odom";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = targetTrajectories.stateTrajectory[i][6];
                pose.pose.position.y = targetTrajectories.stateTrajectory[i][7];
                pose.pose.position.z = 0.05;
                pose.pose.orientation = tf::createQuaternionMsgFromYaw(targetTrajectories.stateTrajectory[i][9]);
                teb_poses.poses.push_back(pose.pose);
            }
            TargetTrajectoriesVisualizerPublisher_.publish(teb_poses);
        }
        rate.sleep();
    } // end of while loop
}

ocs2::scalar_t TargetTrajectoriesPublisher::filter(ocs2::scalar_t &input, ocs2::scalar_t &lastOutput, ocs2::scalar_t alpha) {
    lastOutput = alpha * input + (1 - alpha) * lastOutput;
    input = lastOutput;
    return lastOutput;
}

inline legged_robot::matrix3_t rpyTORotateMat(double roll, double pitch, double yaw) {
    legged_robot::matrix3_t RotateMatrix, R_roll, R_pitch, R_yaw;
    R_roll << 1., 0., 0.,
        0., cos(roll), -sin(roll),
        0., sin(roll), cos(roll);
    R_pitch << cos(pitch), 0, sin(pitch),
        0., 1., 0.,
        -sin(pitch), 0., cos(pitch);
    R_yaw << cos(yaw), -sin(yaw), 0.,
        sin(yaw), cos(yaw), 0.,
        0., 0., 1.;
    RotateMatrix = R_yaw * R_pitch * R_roll;
    return RotateMatrix;
}

TargetTrajectories TargetTrajectoriesPublisher::getTargetTrajectories(const SystemObservation &observation) {
    // get current pose
    const vector_t currentPose = observation.state.segment<6>(6);
    const vector_t currentMometum = observation.state.head(6);

    std::cout << "currentPose " << currentPose.transpose() << std::endl;
    std::cout << "current Time" << observation.time << std::endl;

    // 3x3 transform matrix from odom base to odom frame
    const scalar_t currentYaw = currentPose(3), currentX = currentPose(0), currentY = currentPose(1);

    Eigen::Matrix3d TransformMat;
    TransformMat << cos(currentYaw), -sin(currentYaw), currentX,
        sin(currentYaw), cos(currentYaw), currentY,
        0, 0, 1;
    scalar_array_t timeTrajectory = receivedTargetTrajectories_.timeTrajectory;
    vector_array_t stateTrajectory(timeTrajectory.size(), vector_t::Zero(observation.state.size()));
    const vector_array_t inputTrajectory(timeTrajectory.size(), vector_t::Zero(observation.input.size()));

    // const auto& model = pinocchioInterfacePtr->getModel();
    // auto& data = pinocchioInterfacePtr->getData();

    // const auto& Ag = pinocchio::computeCentroidalMap(model, data, observation.state.tail(18));
    const vector_t q = observation.state.tail(18);
    updateCentroidalDynamics(pinocchioInterface_, centroidalModelInfo_, q);
    const auto currentqVelocity = mappingPtr_->getPinocchioJointVelocity(observation.state, observation.input);
    std::cout << "currentqVelocity: " << currentqVelocity.transpose() << "\n";
    int pointCounter = 0;
    for (const auto &state : receivedTargetTrajectories_.stateTrajectory) {
        Eigen::Vector3d pose_xy;
        pose_xy << state(0), state(1), 1;
        const auto poseInOdomFrame = TransformMat * pose_xy;
        auto targetPose = (vector_t(6) << poseInOdomFrame(0), poseInOdomFrame(1), currentPose(2),
                           currentYaw + state(2), currentPose(4), currentPose(5))
                              .finished();
        stateTrajectory[pointCounter] << vector_t::Zero(6), targetPose, defaultJointState_;
        pointCounter++;
    }

    pointCounter = 0;
    for (const auto& input : receivedTargetTrajectories_.inputTrajectory) {
        Eigen::Vector2d input_xy;
        input_xy << input(0), input(1);
        const auto commandVelInOdomFrame = TransformMat.topLeftCorner(2,2) * input_xy;

        vector_t commandGeneralizedVelocity = vector_t::Zero(18);
        commandGeneralizedVelocity(0) = currentqVelocity(0) + commandVelInOdomFrame(0);  // X 
        commandGeneralizedVelocity(1) = currentqVelocity(1) + commandVelInOdomFrame(1);  // Y 
        commandGeneralizedVelocity(3) = currentqVelocity(3) + input(2);  // Yaw
        // std::cout << "cmd vel:" << input_xy.transpose() << " " << "commandVelInOdomFrame:"
        //     << commandVelInOdomFrame.transpose() << " " 
        //     << "final vel: " << commandGeneralizedVelocity.head(4).transpose() << "\n";

        const auto commandMometum = getCentroidalMomentumMatrix(pinocchioInterface_) * commandGeneralizedVelocity / centroidalModelInfo_.robotMass;
        // std::cout << "commandMometum: " << commandMometum.transpose() << "\n";
        // std::cout << "centroidalModelInfo_.robotMass: " << centroidalModelInfo_.robotMass << "\n";
        stateTrajectory[pointCounter].head(6) = commandMometum;

    }

    // scalar_t timeOffset = receivedTargetTrajectories_.timeTrajectory.front() - observation.time;
    // for (size_t i = 0; i < timeTrajectory.size(); i++) {
    //     timeTrajectory[i] -= timeOffset;
    // }
    

    // ----------------------------------

    // 4x4 transform matrix from odom base to odom frame
    // const scalar_t currentYaw = currentPose(3);
    // matrix_t tfMatrix = matrix_t::Identity(4, 4);
    // const legged_robot::vector3_t ZyxEulerAngles = currentPose.tail(3);
    // std::cout << "ZyxEulerAngles: " << ZyxEulerAngles.transpose() << "\n";
    // legged_robot::matrix3_t rotationMatrix = ocs2::getRotationMatrixFromZyxEulerAngles(ZyxEulerAngles);
    // std::cout << "ocs2::rotationMatrix = \n"
    //           << rotationMatrix << "\n";
    // rotationMatrix = rpyTORotateMat(currentPose(5), currentPose(4), currentPose(3));
    // std::cout << "rpyTORotateMat = \n"
    //           << rotationMatrix << "\n";
    // tfMatrix.topLeftCorner(3, 3) = rotationMatrix;
    // tfMatrix.topRightCorner(3, 1) = currentPose.head(3);

    // scalar_array_t timeTrajectory = receivedTargetTrajectories_.timeTrajectory;
    // vector_array_t stateTrajectory(timeTrajectory.size(), vector_t::Zero(observation.state.size()));
    // const vector_array_t inputTrajectory(timeTrajectory.size(), vector_t::Zero(observation.input.size()));
    // int pointCounter = 0;

    // for (const auto &state : receivedTargetTrajectories_.stateTrajectory) {
    //     legged_robot::vector3_t pose;
    //     pose << state(0), state(1), 0;
    //     const auto poseInOdomFrame = tfMatrix * pose.homogeneous();
    //     auto targetPose = (vector_t(6) << poseInOdomFrame(0), poseInOdomFrame(1), currentPose(2),
    //                        currentYaw + state(2), currentPose(4), currentPose(5))
    //                           .finished();
    //     stateTrajectory[pointCounter] << vector_t::Zero(6), targetPose, defaultJointState_;
    //     pointCounter++;
    // }

    return {timeTrajectory, stateTrajectory, inputTrajectory};
}

} // namespace ocs2
