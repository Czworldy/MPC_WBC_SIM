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

#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include <ros/subscriber.h>

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/command/TargetTrajectoriesRosPublisher.h>
#include <ocs2_msgs/upper_layer_trajectory.h>

namespace ocs2 {

/**
 * This class subscribes target trajectories from upper layer.
 */
class TargetTrajectoriesUpperLayer final {
 public:
  using CommandPolicyToTargetTrajectories =
      std::function<TargetTrajectories(const TargetTrajectories& commandPolicy, const SystemObservation& observation)>;

  /**
   * Constructor
   *
   * @param [in] nodeHandle: ROS node handle.
   * @param [in] topicPrefix: The TargetTrajectories will be published on "topicPrefix_mpc_target" topic. Moreover, the latest
   * observation is be expected on "topicPrefix_mpc_observation" topic.
   * @param [in] targetCommandLimits: The limits of the loaded command from upper layer policy (for safety purposes).
   * @param [in] commandPolicyToTargetTrajectoriesFun: A function which transforms the command line input to TargetTrajectories.
   */
  TargetTrajectoriesUpperLayer (::ros::NodeHandle& nodeHandle, 
                                const std::string& topicPrefix_currentLayer,
                                const std::string& topicPrefix_upperLayer,
                                CommandPolicyToTargetTrajectories commandPolicyToTargetTrajectoriesFun);

  /** Gets the command vector size. */
  size_t targetCommandSize() const { return targetCommandLimits_.size(); }

  void publishCommandPolicyFromUpperLayer();


 private:
  TargetTrajectories readCommandPolicyMsg(const ocs2_msgs::upper_layer_trajectory& commandPolicyMsg);

  const vector_t targetCommandLimits_;
  CommandPolicyToTargetTrajectories commandPolicyToTargetTrajectoriesFun_;

  std::unique_ptr<TargetTrajectoriesRosPublisher> targetTrajectoriesPublisherPtr_;

  ::ros::Subscriber observationSubscriber_;
  mutable std::mutex latestObservationMutex_;
  SystemObservation latestObservation_;

  ::ros::Subscriber commandPolicySubscriber_;
  mutable std::mutex latestCommandPolicyMutex_;
  TargetTrajectories latestCommandPolicy_;


  bool isObservationCome = false;
  bool isMpcPolicyCome = false;
};

}  // namespace ocs2
