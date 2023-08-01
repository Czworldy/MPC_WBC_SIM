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

#include "ocs2_jypro/command/TargetTrajectoriesJoyPublisher.h"

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/Display.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesJoyPublisher::TargetTrajectoriesJoyPublisher(::ros::NodeHandle &nodeHandle, const std::string &topicPrefix,
                                                               const scalar_array_t &targetCommandLimits, const ocs2::scalar_t joyGainLinearFactors, const ocs2::scalar_t joyGainAngularFactors,
                                                               CommandLineToTargetTrajectories commandLineToTargetTrajectoriesFun)
    : targetCommandLimits_(Eigen::Map<const vector_t>(targetCommandLimits.data(), targetCommandLimits.size())),
      joyGainLinearFactors_(joyGainLinearFactors), joyGainAngularFactors_(joyGainAngularFactors),
      commandLineToTargetTrajectoriesFun_(std::move(commandLineToTargetTrajectoriesFun)) {
    // observation subscriber
    auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(latestObservationMutex_);
        latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
        this->isMpcPolicyCome = true;
    };
    observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(topicPrefix + "_mpc_observation", 1, observationCallback);
    // raw joy
    // auto joyCallback = [this](const sensor_msgs::Joy::ConstPtr& msg) {
    //   std::lock_guard<std::mutex> lock(latestJoyMsgsMutex_);

    //   deltaX = msg->axes[1] * joyGainLinearFactors_;
    //   deltaY = msg->axes[0] * joyGainLinearFactors_;
    //   deltaYaw = msg->axes[3] * joyGainAngularFactors_;

    //   filter(deltaX, lastdeltaX, 0.4);
    //   filter(deltaY, lastdeltaY, 0.4);
    //   filter(deltaYaw, la stdeltaYaw, 0.4);
    //   this->isJoyMsgsCome = true;
    // };
    // joySubscriber_ = nodeHandle.subscribe<sensor_msgs::Joy>("joy", 1, joyCallback);

    // vel
    auto joyCallback = [this](const geometry_msgs::Twist::ConstPtr &msg) {
        std::lock_guard<std::mutex> lock(latestJoyMsgsMutex_);

        deltaX = 0.05 + msg->linear.x;
        deltaY = -0.07 + msg->linear.y * joyGainLinearFactors_; // msg->linear.x
        deltaYaw = 0.06 + msg->angular.z * joyGainAngularFactors_;
        this->isJoyMsgsCome = true;
    };

    joySubscriber_ = nodeHandle.subscribe<geometry_msgs::Twist>("/vel", 1, joyCallback);

    // Trajectories publisher
    targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, topicPrefix));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesJoyPublisher::publishKeyboardCommand(const std::string &commadMsg) {
    ::ros::Rate rate(10);
    while (ros::ok() && ros::master::check()) {
        ::ros::spinOnce();
        if (isJoyMsgsCome && isMpcPolicyCome) {
            std::lock_guard<std::mutex> lock(latestJoyMsgsMutex_);
            const Eigen::Matrix<ocs2::scalar_t, 4, 1> commandLineInput = {deltaX, deltaY, 0, deltaYaw};

            // display
            std::cout << "The following command is published: [" << toDelimitedString(commandLineInput) << "]\n\n";

            SystemObservation observation;
            {
                std::lock_guard<std::mutex> lock(latestObservationMutex_);
                observation = latestObservation_;
            }

            // get TargetTrajectories
            const auto targetTrajectories = commandLineToTargetTrajectoriesFun_(commandLineInput, observation);

            // publish TargetTrajectories
            targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
            this->isJoyMsgsCome = false;
        }
        rate.sleep();
    } // end of while loop
}

ocs2::scalar_t TargetTrajectoriesJoyPublisher::filter(ocs2::scalar_t &input, ocs2::scalar_t &lastOutput, ocs2::scalar_t alpha) {
    lastOutput = alpha * input + (1 - alpha) * lastOutput;
    input = lastOutput;
    return lastOutput;
}

} // namespace ocs2
