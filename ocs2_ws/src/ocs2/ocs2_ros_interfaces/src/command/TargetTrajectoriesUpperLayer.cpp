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

#include "ocs2_ros_interfaces/command/TargetTrajectoriesUpperLayer.h"

#include <ocs2_core/misc/Display.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectoriesUpperLayer::TargetTrajectoriesUpperLayer(::ros::NodeHandle& nodeHandle, 
                                                           const std::string& topicPrefix_currentLayer,
                                                           const std::string& topicPrefix_upperLayer,
                                                           CommandPolicyToTargetTrajectories commandPolicyToTargetTrajectoriesFun)
    : commandPolicyToTargetTrajectoriesFun_(std::move(commandPolicyToTargetTrajectoriesFun)) {

  // observation subscriber
  auto observationCallback = [this](const ocs2_msgs::mpc_observation::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(latestObservationMutex_);
    latestObservation_ = ros_msg_conversions::readObservationMsg(*msg);
    this->isObservationCome = true;
  };
  observationSubscriber_ = nodeHandle.subscribe<ocs2_msgs::mpc_observation>(topicPrefix_currentLayer + "_mpc_observation", 1, observationCallback);

  // command policy subscriber
  auto commandPolicyCallback = [this](const ocs2_msgs::upper_layer_trajectory::ConstPtr& msg) {
    std::cout << "I RECEIVE THE MSG!" << std::endl;
    std::lock_guard<std::mutex> lock(latestCommandPolicyMutex_);
    latestCommandPolicy_ = readCommandPolicyMsg(*msg);
    this->isMpcPolicyCome = true;
  };
  commandPolicySubscriber_ = nodeHandle.subscribe<ocs2_msgs::upper_layer_trajectory>("/mobile_manipulator_upper_layer_policy", 1, commandPolicyCallback);

  // Trajectories publisher
  targetTrajectoriesPublisherPtr_.reset(new TargetTrajectoriesRosPublisher(nodeHandle, topicPrefix_currentLayer));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void TargetTrajectoriesUpperLayer::publishCommandPolicyFromUpperLayer() {
  ::ros::Rate rate(50);

  while (ros::ok() && ros::master::check()) {

    if(isMpcPolicyCome && isObservationCome) {
      // get the latest observation
      SystemObservation observation;
      {
        std::lock_guard<std::mutex> lock(latestObservationMutex_);
        observation = latestObservation_;
      }
      // get the latest command policy
      TargetTrajectories commandPolicy;
      {
        std::lock_guard<std::mutex> lock(latestCommandPolicyMutex_);
        commandPolicy = latestCommandPolicy_;
      }

      // get the targetTrajectories
      const auto targetTrajectories = commandPolicyToTargetTrajectoriesFun_(commandPolicy, observation);

      // publish TargetTrajectories
      targetTrajectoriesPublisherPtr_->publishTargetTrajectories(targetTrajectories);
    }

    std::cout << "isMpcPolicyCome " << isMpcPolicyCome << std::endl;
    std::cout << "isObservationCome " << isObservationCome << std::endl;
    
    ::ros::spinOnce();
    rate.sleep();
  } // end of while loop
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
TargetTrajectories TargetTrajectoriesUpperLayer::readCommandPolicyMsg(const ocs2_msgs::upper_layer_trajectory& commandPolicyMsg) {

  if (commandPolicyMsg.timeTrajectory.size() < 1) {
    throw std::runtime_error("[TargetTrajectoriesUpperLayer::readCommandPolicyMsg] Command Policy is blank. Aborting.");
  }

  size_t N_times(commandPolicyMsg.timeTrajectory.size());
  size_t stateDim(commandPolicyMsg.stateTrajectory[0].value.size());
  size_t inputDim(commandPolicyMsg.inputTrajectory[0].value.size());
  scalar_array_t timeTrajectory(N_times);
  vector_array_t stateTrajectory(N_times, vector_t::Zero(stateDim));
  vector_array_t inputTrajectory(N_times, vector_t::Zero(inputDim));

  for (int i(0); i<N_times; i++) {
    // copy time
    timeTrajectory[i] = commandPolicyMsg.timeTrajectory[i];
    // copy state vector
    for (int j(0); j<stateDim; j++) {
      stateTrajectory[i][j] = commandPolicyMsg.stateTrajectory[i].value[j];
    }

    //copy input vector
    for (int k(0); k<inputDim; k++) {
      inputTrajectory[i][k] = commandPolicyMsg.inputTrajectory[i].value[k];
    }
  } // end of N_time loop

  return {timeTrajectory, stateTrajectory, inputTrajectory};
} 

}  // namespace ocs2
