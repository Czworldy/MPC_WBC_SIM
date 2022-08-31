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
                                                             std::shared_ptr<FootPlacementPlanner> footPlacementPlannerPtr)
    : LeggedRobotReferenceManager(TargetTrajectories(), ModeSchedule(), TargetFeetPlacement()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
      footPlacementPlannerPtr_(std::move(footPlacementPlannerPtr)) {}

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
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  std::cout << "init time:" << initTime<< "\t" << " final time:" << finalTime << std::endl;
  std::cout << modeSchedule << std::endl;

  const scalar_t terrainHeight = initState(8) - 0.42; //For JYPro

  std::cout << "targetFeetPlacement L size:" << targetFeetPlacement.targetFeetPlacemetLeft_.size() << "\n";
  std::cout << "targetFeetPlacement R size:" << targetFeetPlacement.targetFeetPlacemetRight_.size() << "\n";
  const auto& left = targetFeetPlacement.targetFeetPlacemetLeft_;
  const auto& right = targetFeetPlacement.targetFeetPlacemetRight_;
  for(const auto& left_i : left) {
    std::cout << "left_i:" << left_i.transpose() << "\n";
  }
  for(const auto& right_i : right) {
    std::cout << "right_i:" << right_i.transpose() << "\n";
  }
  footPlacementPlannerPtr_->setTargetPoints(left, right);

  footPlacementPlannerPtr_->update(modeSchedule, targetTrajectories, initTime, initState);
  
  // Normal swing feet trajectory
  swingTrajectoryPtr_->update(modeSchedule, terrainHeight);



  // For terrain aware swing feet trajectory planning
  // swingTrajectoryPtr_->update(modeSchedule, 
  //                               footPlacementPlannerPtr_->getliftOffHeightSequence(), 
  //                               footPlacementPlannerPtr_->gettouchDownHeightSequence(),
  //                               footPlacementPlannerPtr_->getfeetPlacementEvents(), initTime);
  
}

}  // namespace legged_robot
}  // namespace ocs2
