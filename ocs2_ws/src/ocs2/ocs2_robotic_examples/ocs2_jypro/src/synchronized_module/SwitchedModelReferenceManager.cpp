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

#include "ocs2_jypro/synchronized_module/SwitchedModelReferenceManager.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                             std::shared_ptr<FootPlacementPlanner> footPlacementPlannerPtr,
                                                             std::shared_ptr<TerrainEstData> terrainEstDataPtr)
    : LeggedRobotReferenceManager(TargetTrajectories(), ModeSchedule(), TargetFeetPlacement()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
      footPlacementPlannerPtr_(std::move(footPlacementPlannerPtr)),
      terrainEstDataPtr_(std::move(terrainEstDataPtr)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  std::cout << "init time:" << initTime<< "\t" << " final time:" << finalTime << std::endl;
  std::cout << modeSchedule << std::endl;

  const scalar_t terrainHeight = initState(8) - 0.42; //For JYPro

  footPlacementPlannerPtr_->update(modeSchedule, targetTrajectories, initTime, initState);
  
  // Normal swing feet trajectory
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);

  // For terrain aware swing feet trajectory planning
  // swingTrajectoryPtr_->update(modeSchedule, 
  //                               footPlacementPlannerPtr_->getliftOffHeightSequence(), 
  //                               footPlacementPlannerPtr_->gettouchDownHeightSequence(),
  //                               footPlacementPlannerPtr_->getfeetPlacementEvents(), initTime);
  
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule,
                                                     TargetFeetPlacement& targetFeetPlacement) {
  const auto timeHorizon = finalTime - initTime;
  std::cout << "init time:" << initTime<< "\t" << " final time:" << finalTime << std::endl;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  auto& targetState = targetTrajectories.stateTrajectory.back();
  const vector3_t& terrainParams = terrainEstDataPtr_->terrainParams.cast<scalar_t>();
  vector3_t terrainNormal = terrainParams;
  terrainNormal(2) = 1;

  vector3_t terrainRPY = terrainQuaternionToRPY_.quaternionToTotalRad(terrainEstDataPtr_->terrainQuat.cast<scalar_t>());
  // std::cout << "terrainRPY: " << terrainRPY.transpose() << std::endl;

  const scalar_t distance2Terrain = 0.48; //For X20
  const scalar_t D2 = terrainParams[2] - distance2Terrain * terrainNormal.norm(); // D1 - h*sqrt(A^2 + B^2 + 1)
  const scalar_t zReference = - (terrainParams(0) * initState(6) + terrainParams(1) * initState(7) + D2);
  std::cout << "zReference: " << zReference << "\t D2: " << D2 << " terrainNormal.norm: " << terrainNormal.norm() << std::endl;
  // std::cout << "intiState: " << initState.segment(6, 18).transpose() << std::endl;

  if(targetTrajectories.timeTrajectory.size() >= 2){
    // targetState(8) = zReference;
    // targetState(10) = terrainRPY[1]; //pitch
    // targetState(11) = terrainRPY[0]; //roll
    for(auto& stateTrajectory : targetTrajectories.stateTrajectory){
      stateTrajectory(8) = zReference;
      stateTrajectory(10) = terrainRPY[1]; //pitch
      stateTrajectory(11) = terrainRPY[0]; //roll
    }
  }

  //   std::cout << "######## modify target state ########\n"; 
  // }

  // std::cout << "targetTrajectories:" << targetTrajectories.stateTrajectory.back().segment(6,6).transpose() << std::endl;
 
  std::cout << "init time:" << initTime<< "\t" << " final time:" << finalTime << std::endl;
  std::cout << modeSchedule << std::endl;

  // const scalar_t terrainHeight = initState(8) - 0.42; //For JYPro

  // std::cout << "targetFeetPlacement L size:" << targetFeetPlacement.targetFeetPlacemetLeft_.size() << "\n";
  // std::cout << "targetFeetPlacement R size:" << targetFeetPlacement.targetFeetPlacemetRight_.size() << "\n";
  const auto& leftFront = targetFeetPlacement.targetFeetPlacemetLeftFront_;
  const auto& rightFront = targetFeetPlacement.targetFeetPlacemetRightFront_;
  const auto& leftBack = targetFeetPlacement.targetFeetPlacemetLeftBack_;
  const auto& rightBack = targetFeetPlacement.targetFeetPlacemetRightBack_;
  // for(const auto& left_i : leftFront) {
  //   std::cout << "left_i:" << left_i.transpose() << "\n";
  // }
  // for(const auto& right_i : right) {
  //   std::cout << "right_i:" << right_i.transpose() << "\n";
  // }
  footPlacementPlannerPtr_->setTargetPoints(leftFront, rightFront, leftBack, rightBack);

  footPlacementPlannerPtr_->update(modeSchedule, targetTrajectories, initTime, initState);
  
  // Normal swing feet trajectory
  // swingTrajectoryPtr_->update(modeSchedule, terrainEstDataPtr_->feetHeight.cast<scalar_t>());
  swingTrajectoryPtr_->update(modeSchedule, footPlacementPlannerPtr_->getfeetPlacement());
  // swingTrajectoryPtr_->update(modeSchedule, 0.03);


  // std::cout << *terrainEstDataPtr_ << std::endl;
  // For terrain aware swing feet trajectory planning
  // this part makes the robot performance worse. something wrong in this part.
  // swingTrajectoryPtr_->update(modeSchedule, 
  //                               footPlacementPlannerPtr_->getliftOffHeightSequence(), 
  //                               footPlacementPlannerPtr_->gettouchDownHeightSequence(),
  //                               footPlacementPlannerPtr_->getfeetPlacementEvents(), initTime);

  std::cout << "modifyReferences Done!" << "\n";
  
}

}  // namespace legged_robot
}  // namespace ocs2
