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

#pragma once

#include <ocs2_core/thread_support/Synchronized.h>

#include "ocs2_jypro/synchronized_module/LeggedRobotReferenceManager.h"
#include "ocs2_jypro/synchronized_module/TerrainReceiver.h"
#include "ocs2_jypro/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_jypro/foot_planner/FootPlacementPlanner.h"
#include "ocs2_jypro/gait/GaitSchedule.h"
#include "ocs2_jypro/gait/MotionPhaseDefinition.h"
#include "ocs2_jypro/BodyPositionEstimator/BodyPositionEstimator.h"

namespace ocs2 {
namespace legged_robot {

/**
 * Manages the ModeSchedule and the TargetTrajectories for switched model.
 */
class SwitchedModelReferenceManager : public LeggedRobotReferenceManager {
 public:
  SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr, std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                std::shared_ptr<FootPlacementPlanner> footPlacementPlannerPtr,
                                std::shared_ptr<TerrainEstData> terrainEstDataPtr);

  ~SwitchedModelReferenceManager() override = default;

  contact_flag_t getContactFlags(scalar_t time) const;

  const std::shared_ptr<GaitSchedule>& getGaitSchedule() { return gaitSchedulePtr_; }

  const std::shared_ptr<SwingTrajectoryPlanner>& getSwingTrajectoryPlanner() { return swingTrajectoryPtr_; }

  const std::shared_ptr<FootPlacementPlanner>& getFootPlacementPlanner() { return footPlacementPlannerPtr_; }

  std::shared_ptr<TerrainEstData>& getTerrainEstDataPtr() { return terrainEstDataPtr_; }

 private:
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule) override;
  void modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState, TargetTrajectories& targetTrajectories,
                        ModeSchedule& modeSchedule, TargetFeetPlacement& targetFeetPlacement) override;

  std::shared_ptr<GaitSchedule> gaitSchedulePtr_;
  std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr_;
  std::shared_ptr<FootPlacementPlanner> footPlacementPlannerPtr_;
  std::shared_ptr<TerrainEstData> terrainEstDataPtr_;
  QuaternionToRPY terrainQuaternionToRPY_;
};

}  // namespace legged_robot
}  // namespace ocs2
