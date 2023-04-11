/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include <ros/init.h>
#include <urdf_parser/urdf_parser.h>

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ros_interfaces/mpc/MPC_ROS_Interface.h>

#include <ocs2_jypro/synchronized_module/LeggedRobotRosReferenceManager.h>
#include "ocs2_jypro/LeggedRobotInterface.h"
#include "ocs2_jypro/gait/GaitReceiver.h"
#include "ocs2_jypro/synchronized_module/TerrainReceiver.h"

#include <ocs2_sqp/MultipleShootingMpc.h>
#include "ocs2_jypro/visualization/FootPlacementVisualizer.h"
#include "ocs2_jypro/synchronized_module/LegEndEffectorsPolygonReceiver.h"

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
  ocs2::legged_robot::LeggedRobotInterface interface(taskFile, urdfFile, referenceFile);


  // Gait receiver
  auto gaitReceiverPtr = std::make_shared<ocs2::legged_robot::GaitReceiver>(
      nodeHandle, interface.getSwitchedModelReferenceManagerPtr()->getGaitSchedule(), robotName);

  // ROS ReferenceManager
  auto rosReferenceManagerPtr = std::make_shared<ocs2::LeggedRobotRosReferenceManager>(robotName, interface.getSwitchedModelReferenceManagerPtr());
  rosReferenceManagerPtr->subscribe(nodeHandle);

  // Terrain receiver
  auto terrainReceiverPtr = std::make_shared<ocs2::legged_robot::TerrainReceiver>(nodeHandle,
      interface.getSwitchedModelReferenceManagerPtr()->getTerrainEstDataPtr(), robotName);

  auto footPlacementPublisher = std::make_shared<ocs2::legged_robot::FootPlacementVisualizer>(nodeHandle, *interface.getSwitchedModelReferenceManagerPtr()->getSwingTrajectoryPlanner());

  auto polygonReceiverPtr = std::make_shared<ocs2::legged_robot::LegEndEffectorsPolygonReceiver>
                            (nodeHandle, interface.getSwitchedModelReferenceManagerPtr()->getMpcPolygonArrayPtr(), 
                            interface.getSwitchedModelReferenceManagerPtr()->getMpcNominalFeetholdsPtr(),
                              robotName);
  // MPC
  // ocs2::GaussNewtonDDP_MPC mpc(interface.mpcSettings(), interface.ddpSettings(), interface.getRollout(), interface.getOptimalControlProblem(),
  //                   interface.getInitializer());
  ocs2::MultipleShootingMpc mpc(interface.mpcSettings(), interface.sqpSettings(), interface.getOptimalControlProblem(),
                        interface.getInitializer());
  mpc.getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);  //for perRun
  mpc.getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);       //for preRun
  mpc.getSolverPtr()->addSynchronizedModule(terrainReceiverPtr);       //for preRun
  mpc.getSolverPtr()->addSynchronizedModule(footPlacementPublisher);       //for preRun
  mpc.getSolverPtr()->addSynchronizedModule(polygonReceiverPtr);

  // Launch MPC ROS node
  ocs2::MPC_ROS_Interface mpcNode(mpc, robotName);
  mpcNode.launchNodes(nodeHandle);

  // Successful exit
  return 0;
}
