/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

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

#include <string>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LoadData.h>

#include <ocs2_jypro/command/TargetTrajectoriesPublisher.h>
#include "sensor_msgs/Joy.h"

using namespace ocs2;

namespace {
scalar_t targetDisplacementVelocity;
scalar_t targetRotationVelocity;
scalar_t comHeight;
vector_t defaultJointState(12);
} // namespace





int main(int argc, char *argv[]) {
    // ros node handle
    const std::string robotName = "legged_robot";

    ::ros::init(argc, argv, robotName + "_target_joy");
    ::ros::NodeHandle nodeHandle;
    std::string targetCommandFile;
    nodeHandle.getParam("/referenceFile", targetCommandFile);
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(targetCommandFile, pt);
    // targetDisplacementVelocity = pt.get<scalar_t>("targetDisplacementVelocity");
    // targetRotationVelocity = pt.get<scalar_t>("targetRotationVelocity");
    // comHeight = pt.get<scalar_t>("comHeight");
    ocs2::loadData::loadEigenMatrix(targetCommandFile, "defaultJointState", defaultJointState);

    // ocs2::scalar_t joyLinearVelocityGain = pt.get<scalar_t>("joyLinearVelocityGain");
    // ocs2::scalar_t joyRotationVelocityGain = pt.get<scalar_t>("joyRotationVelocityGain");

    std::cout << "defaultJointState: " << defaultJointState.transpose() << std::endl;

    // goalPose: [deltaX, deltaY, deltaZ, deltaYaw]
    TargetTrajectoriesPublisher targetPoseCommand(nodeHandle, robotName, defaultJointState);

    const std::string commandMsg = "Enter XYZ and Yaw (deg) displacements for the TORSO, separated by spaces";
    targetPoseCommand.publishKeyboardCommand(commandMsg);

    // Successful exit
    return 0;
}
