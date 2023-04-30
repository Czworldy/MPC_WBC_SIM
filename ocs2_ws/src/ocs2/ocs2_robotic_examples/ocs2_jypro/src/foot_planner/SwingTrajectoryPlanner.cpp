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

#include "ocs2_jypro/foot_planner/SwingTrajectoryPlanner.h"

#include <ocs2_core/misc/Lookup.h>

#include "ocs2_jypro/gait/MotionPhaseDefinition.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwingTrajectoryPlanner::SwingTrajectoryPlanner(Config config, size_t numFeet) : config_(std::move(config)), numFeet_(numFeet) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetHeightTrajectories_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetHeightTrajectories_[leg][index].position(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getXvelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetXTrajectories_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getXpositionConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetXTrajectories_[leg][index].position(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getYvelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetYTrajectories_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getYpositionConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetYTrajectories_[leg][index].position(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getSwingTimeLeft(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  return feetHeightTrajectoriesEvents_[leg][index] - time;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, scalar_t terrainHeight) {
  const scalar_array_t terrainHeightSequence(modeSchedule.modeSequence.size(), terrainHeight);
  // const scalar_array_t terrainHeightSequence_(modeSchedule.modeSequence.size(), terrainHeight+0.1);
  feet_array_t<scalar_array_t> liftOffHeightSequence;
  liftOffHeightSequence.fill(terrainHeightSequence);
  feet_array_t<scalar_array_t> touchDownHeightSequence;
  touchDownHeightSequence.fill(terrainHeightSequence);
  update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, const vector_t& terrainHeight) {
  const scalar_array_t terrainHeightSequenceLF(modeSchedule.modeSequence.size(), terrainHeight[0]);
  const scalar_array_t terrainHeightSequenceLH(modeSchedule.modeSequence.size(), terrainHeight[1]);
  const scalar_array_t terrainHeightSequenceRF(modeSchedule.modeSequence.size(), terrainHeight[2]);
  const scalar_array_t terrainHeightSequenceRH(modeSchedule.modeSequence.size(), terrainHeight[3]);
  // const scalar_array_t terrainHeightSequence_(modeSchedule.modeSequence.size(), terrainHeight+0.1);
  feet_array_t<scalar_array_t> liftOffHeightSequence;  //{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"}
  liftOffHeightSequence[0] = terrainHeightSequenceLF;
  liftOffHeightSequence[1] = terrainHeightSequenceRF;
  liftOffHeightSequence[2] = terrainHeightSequenceLH;
  liftOffHeightSequence[3] = terrainHeightSequenceRH;


  feet_array_t<scalar_array_t> touchDownHeightSequence = liftOffHeightSequence;
  // touchDownHeightSequence.fill(terrainHeightSequence);
  update(modeSchedule, liftOffHeightSequence, touchDownHeightSequence);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, const feet_array_t<scalar_array_t>& liftOffHeightSequence,
                                    const feet_array_t<scalar_array_t>& touchDownHeightSequence) {
  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;

  const auto eesContactFlagStocks = extractContactFlags(modeSequence);

  feet_array_t<std::vector<int>> startTimesIndices;
  feet_array_t<std::vector<int>> finalTimesIndices;
  for (size_t leg = 0; leg < numFeet_; leg++) {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
  }

  for (size_t j = 0; j < numFeet_; j++) {
    feetHeightTrajectories_[j].clear();
    feetHeightTrajectories_[j].reserve(modeSequence.size());
    for (int p = 0; p < modeSequence.size(); ++p) {
      if (!eesContactFlagStocks[j][p]) {  // for a swing leg
        const int swingStartIndex = startTimesIndices[j][p];
        const int swingFinalIndex = finalTimesIndices[j][p];
        checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

        const scalar_t swingStartTime = eventTimes[swingStartIndex];
        const scalar_t swingFinalTime = eventTimes[swingFinalIndex];

        const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);

        const CubicSpline::Node liftOff{swingStartTime, liftOffHeightSequence[j][p], scaling * config_.liftOffVelocity};
        const CubicSpline::Node touchDown{swingFinalTime, touchDownHeightSequence[j][p], scaling * config_.touchDownVelocity};
        const scalar_t midHeight = std::min(liftOffHeightSequence[j][p], touchDownHeightSequence[j][p]) + scaling * config_.swingHeight;
        feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);
      } else {  // for a stance leg
        const CubicSpline::Node liftOff{0.0, liftOffHeightSequence[j][p], 0.0};
        const CubicSpline::Node touchDown{1.0, liftOffHeightSequence[j][p], 0.0};
        feetHeightTrajectories_[j].emplace_back(liftOff, liftOffHeightSequence[j][p], touchDown);
      }
    }
    feetHeightTrajectoriesEvents_[j] = eventTimes;
  }
}


/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, const feet_array_t<scalar_array_t>& liftOffHeightSequence,
                                    const feet_array_t<scalar_array_t>& touchDownHeightSequence, 
                                    const feet_array_t<scalar_array_t>& feetHeightTrajectoriesEvents,
                                    scalar_t initTime) {
  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;

  const size_t initIndex = lookup::findIndexInTimeArray(eventTimes, initTime);

  const auto eesContactFlagStocks = extractContactFlags(modeSequence);

  //this events times is copy from foot placement constraints to make sure 
  //that the events times are the same as the foot placement constraints
  //because when a foot is in stance, the events times deesn't change
  feetHeightTrajectoriesEvents_ = feetHeightTrajectoriesEvents;

  feet_array_t<std::vector<int>> startTimesIndices;
  feet_array_t<std::vector<int>> finalTimesIndices;
  for (size_t leg = 0; leg < numFeet_; leg++) {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
  }

  for (size_t j = 0; j < numFeet_; j++) {
    if (eesContactFlagStocks[j][initIndex]){  //only for current stance feet ? //TODO can this line 
      feetHeightTrajectories_[j].clear();
      feetHeightTrajectories_[j].reserve(feetHeightTrajectoriesEvents_[j].size());
      for (int p = 0; p < feetHeightTrajectoriesEvents_[j].size(); ++p) { // use the events times to update the foot trajectories
        if (!eesContactFlagStocks[j][p]) {  // for a swing leg
          const int swingStartIndex = startTimesIndices[j][p];
          const int swingFinalIndex = finalTimesIndices[j][p];
          checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

          const scalar_t swingStartTime = feetHeightTrajectoriesEvents_[j][swingStartIndex];
          const scalar_t swingFinalTime = feetHeightTrajectoriesEvents_[j][swingFinalIndex];

          const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);

          const CubicSpline::Node liftOff{swingStartTime, liftOffHeightSequence[j][p], scaling * config_.liftOffVelocity};
          const CubicSpline::Node touchDown{swingFinalTime, touchDownHeightSequence[j][p], scaling * config_.touchDownVelocity};
          const scalar_t midHeight = std::min(liftOffHeightSequence[j][p], touchDownHeightSequence[j][p]) + scaling * config_.swingHeight;
          feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);
        } else {  // for a stance leg
          const CubicSpline::Node liftOff{0.0, liftOffHeightSequence[j][p], 0.0};
          const CubicSpline::Node touchDown{1.0, liftOffHeightSequence[j][p], 0.0};
          feetHeightTrajectories_[j].emplace_back(liftOff, liftOffHeightSequence[j][p], touchDown);
        }
      }
    // feetHeightTrajectoriesEvents_[j] = eventTimes;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, const feet_array_t<std::vector<vector3_t>>& feetPlacement, 
                                    scalar_t initTime, const feet_array_t<vector3_t>& currentFeetEndEffectors,
                                    const feet_array_t<std::vector<scalar_t>>& swingHeightSequence, bool isLateTouchdown) {
  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;

  const auto eesContactFlagStocks = extractContactFlags(modeSequence);

  const size_t initIndex = lookup::findIndexInTimeArray(eventTimes, initTime);


  feet_array_t<std::vector<int>> startTimesIndices;
  feet_array_t<std::vector<int>> finalTimesIndices;
  for (size_t leg = 0; leg < numFeet_; leg++) {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
  }

  for (size_t j = 0; j < numFeet_; j++) {
    if (eesContactFlagStocks[j][initIndex]){
    feetHeightTrajectories_[j].clear();
    feetHeightTrajectories_[j].reserve(modeSequence.size());

    feetXTrajectories_[j].clear();
    feetXTrajectories_[j].reserve(modeSequence.size());

    feetYTrajectories_[j].clear();
    feetYTrajectories_[j].reserve(modeSequence.size());
    for (int p = 0; p < modeSequence.size(); ++p) {
      if (!eesContactFlagStocks[j][p]) {  // for a swing leg
        //TODO consider after swing phase another swing phase again.
        int m = p;
        for(; m >= 0; m--){
          if(eesContactFlagStocks[j][m]){
            break;
          }
        }
        // if(j == 0){
        //   std::cout << "p = " << p << " m = " << m << std::endl;
        // }
        const int swingStartIndex = startTimesIndices[j][p];
        const int swingFinalIndex = finalTimesIndices[j][p];
        checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

        const scalar_t swingStartTime = eventTimes[swingStartIndex];
        const scalar_t swingFinalTime = eventTimes[swingFinalIndex];

        const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);

        scalar_t swingHeight = std::max(swingHeightSequence[j][p] + 0.08, 0.12);
        swingHeight = std::min(swingHeight, 0.2);
        // const scalar_t swingHeight = 0.25;
        // std::cout << "feetPlacement[j][p]: " << j << " " << "p" <<  feetPlacement[j][p].transpose() << std::endl;
        

        if (p >= 1){
          // if(p == initIndex){ //Doesn't seem like a good idea to use current end-effortor.
          //   const CubicSpline::Node liftOff{swingStartTime, currentFeetEndEffectors[j].z(), scaling * config_.liftOffVelocity};
          //   const CubicSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), scaling * config_.touchDownVelocity};
          //   const scalar_t midHeight = std::min(currentFeetEndEffectors[j].z(), feetPlacement[j][p].z()) + scaling * config_.swingHeight;
          //   feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);

          //   const CubicSpline::Node xStart{swingStartTime, currentFeetEndEffectors[j].x(), scaling * config_.liftOffVelocity};
          //   const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), scaling * config_.touchDownVelocity};
          //   feetXTrajectories_[j].emplace_back(xStart, xEnd);

          //   const CubicSpline::Node yStart{swingStartTime, currentFeetEndEffectors[j].y(), scaling * config_.liftOffVelocity};
          //   const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), scaling * config_.touchDownVelocity};
          //   feetYTrajectories_[j].emplace_back(yStart, yEnd);
          // } else { 
          // if (p == initIndex && isLateTouchdown) {
          //   /* code */
          //   std::cout << "Late touchdown in phase: " << initIndex << std::endl;
          //   const CubicSpline::Node liftOff{swingStartTime, currentFeetEndEffectors[j].z(), scaling * config_.liftOffVelocity};
          //   const CubicSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), -1.0};
          //   const scalar_t midHeight = std::min(currentFeetEndEffectors[j].z(), feetPlacement[j][p].z()) + scaling * config_.swingHeight;
          //   feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);

          //   const CubicSpline::Node xStart{swingStartTime, currentFeetEndEffectors[j].x(), 0};
          //   const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0};
          //   feetXTrajectories_[j].emplace_back(xStart, xEnd);

          //   const CubicSpline::Node yStart{swingStartTime, currentFeetEndEffectors[j].y(), 0};
          //   const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0};
          //   feetYTrajectories_[j].emplace_back(yStart, yEnd);
          // }
          if (p == initIndex + 1) {
            /* code */
            const CubicSpline::Node liftOff{swingStartTime, currentFeetEndEffectors[j].z(), scaling * config_.liftOffVelocity};
            const CubicSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), config_.touchDownVelocity};
            const scalar_t midHeight = std::min(currentFeetEndEffectors[j].z(), feetPlacement[j][p].z()) + scaling * swingHeight;
            feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);

            const CubicSpline::Node xStart{swingStartTime, currentFeetEndEffectors[j].x(), config_.liftOffVelocity};
            const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0.};
            feetXTrajectories_[j].emplace_back(xStart, xEnd);

            const CubicSpline::Node yStart{swingStartTime, currentFeetEndEffectors[j].y(), config_.liftOffVelocity};
            const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0.};
            feetYTrajectories_[j].emplace_back(yStart, yEnd);
          }
          else{
          const CubicSpline::Node liftOff{swingStartTime, feetPlacement[j][m].z(), scaling * config_.liftOffVelocity};
          const CubicSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), scaling * config_.touchDownVelocity};
          // const scalar_t midHeight = std::min(feetPlacement[j][p-1].z(), feetPlacement[j][p].z()) + scaling * swingHeight;
          const scalar_t midHeight = std::min(feetPlacement[j][p-1].z(), feetPlacement[j][p].z()) + scaling * swingHeight;
          feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);

          const CubicSpline::Node xStart{swingStartTime, feetPlacement[j][m].x(), scaling * config_.liftOffVelocity};
          const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), scaling * 0.};
          feetXTrajectories_[j].emplace_back(xStart, xEnd);

          const CubicSpline::Node yStart{swingStartTime, feetPlacement[j][m].y(), scaling * config_.liftOffVelocity};
          const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), scaling * 0.};
          feetYTrajectories_[j].emplace_back(yStart, yEnd);
          }
        }
        else{
          const CubicSpline::Node liftOff{swingStartTime, feetPlacement[j][p].z(), scaling * config_.liftOffVelocity};
          const CubicSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), scaling * config_.touchDownVelocity};
          // const scalar_t midHeight = std::min(feetPlacement[j][p-1].z(), feetPlacement[j][p].z()) + scaling * swingHeight;
          const scalar_t midHeight = std::min(feetPlacement[j][p-1].z(), feetPlacement[j][p].z()) + scaling * swingHeight;
          feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);

          const CubicSpline::Node xStart{swingStartTime, feetPlacement[j][p].x(), scaling * config_.liftOffVelocity};
          const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), scaling * 0.};
          feetXTrajectories_[j].emplace_back(xStart, xEnd);

          const CubicSpline::Node yStart{swingStartTime, feetPlacement[j][p].y(), scaling * config_.liftOffVelocity};
          const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), scaling * 0.};
          feetYTrajectories_[j].emplace_back(yStart, yEnd);
        }
        
      } else {  // for a stance leg
        const CubicSpline::Node liftOff{0.0, feetPlacement[j][p].z(), 0.0};
        const CubicSpline::Node touchDown{1.0, feetPlacement[j][p].z(), 0.0};
        feetHeightTrajectories_[j].emplace_back(liftOff, feetPlacement[j][p].z(), touchDown);

        const CubicSpline::Node xStart{0.0, feetPlacement[j][p].x(), 0.0};
        const CubicSpline::Node xEnd{1.0, feetPlacement[j][p].x(), 0.0};
        feetXTrajectories_[j].emplace_back(xStart, xEnd);

        const CubicSpline::Node yStart{0.0, feetPlacement[j][p].y(), 0.0};
        const CubicSpline::Node yEnd{1.0, feetPlacement[j][p].y(), 0.0};
        feetYTrajectories_[j].emplace_back(yStart, yEnd);
      }
    }
    // if(j == 0){
    //     for(const auto& p:feetPlacement[j]){
    //       std::cout << "leg: " << j << " x: " << p.x() << " y: " << p.y() << " z: " << p.z() << std::endl;
    //     }
    // }
    
    feetHeightTrajectoriesEvents_[j] = eventTimes;
  }
  }
}

void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, const feet_array_t<vector3_t>& currentFeetEndEffectors,
              scalar_t initTime, const feet_array_t<std::vector<vector3_t>>& targetFeetEndEffectors) {
    const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;

  const auto eesContactFlagStocks = extractContactFlags(modeSequence);

  const size_t initIndex = lookup::findIndexInTimeArray(eventTimes, initTime);


  feet_array_t<std::vector<int>> startTimesIndices;
  feet_array_t<std::vector<int>> finalTimesIndices;
  for (size_t leg = 0; leg < numFeet_; leg++) {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
  }

  for (size_t j = 0; j < numFeet_; j++) {
    feetHeightTrajectories_[j].clear();
    feetHeightTrajectories_[j].reserve(modeSequence.size());

    feetXTrajectories_[j].clear();
    feetXTrajectories_[j].reserve(modeSequence.size());

    feetYTrajectories_[j].clear();
    feetYTrajectories_[j].reserve(modeSequence.size());
    for (int p = 0; p < modeSequence.size(); ++p) {
      if (!eesContactFlagStocks[j][p]) {  // for a swing leg
          //TODO consider after swing phase another swing phase again.
          int m = p;
          for(; m >= 0; m--){
            if(eesContactFlagStocks[j][m]){
              break;
            }
          }


          const int swingStartIndex = startTimesIndices[j][p];
          const int swingFinalIndex = finalTimesIndices[j][p];
          checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

          const scalar_t swingStartTime = eventTimes[swingStartIndex];
          const scalar_t swingFinalTime = eventTimes[swingFinalIndex];

          const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);

          if(p > initIndex && m > initIndex){ // next swing phase
            const CubicSpline::Node liftOff{swingStartTime, targetFeetEndEffectors[j].front().z(), scaling * config_.liftOffVelocity};
            const CubicSpline::Node touchDown{swingFinalTime, targetFeetEndEffectors[j].back().z(), scaling * config_.touchDownVelocity};
            const scalar_t midHeight = std::min(targetFeetEndEffectors[j].front().z(), targetFeetEndEffectors[j].back().z()) + scaling * config_.swingHeight;
            feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);

            const CubicSpline::Node xStart{swingStartTime, targetFeetEndEffectors[j].front().x(), scaling * config_.liftOffVelocity};
            const CubicSpline::Node xEnd{swingFinalTime, targetFeetEndEffectors[j].back().x(), scaling * config_.touchDownVelocity};
            feetXTrajectories_[j].emplace_back(xStart, xEnd);

            const CubicSpline::Node yStart{swingStartTime, targetFeetEndEffectors[j].front().y(), scaling * config_.liftOffVelocity};
            const CubicSpline::Node yEnd{swingFinalTime, targetFeetEndEffectors[j].back().y(), scaling * config_.touchDownVelocity};
            feetYTrajectories_[j].emplace_back(yStart, yEnd);
          }
          else{
            const CubicSpline::Node liftOff{swingStartTime, currentFeetEndEffectors[j].z(), scaling * config_.liftOffVelocity};
            const CubicSpline::Node touchDown{swingFinalTime, targetFeetEndEffectors[j].front().z(), scaling * config_.touchDownVelocity};
            // const scalar_t midHeight = std::min(currentFeetEndEffectors[j].z(), targetFeetEndEffectors[j].front().z()) + scaling * config_.swingHeight;
            const scalar_t midHeight =  targetFeetEndEffectors[j].front().z() + scaling * config_.swingHeight;
            feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);

            const CubicSpline::Node xStart{swingStartTime, currentFeetEndEffectors[j].x(), scaling * config_.liftOffVelocity};
            const CubicSpline::Node xEnd{swingFinalTime, targetFeetEndEffectors[j].front().x(), scaling * config_.touchDownVelocity};
            feetXTrajectories_[j].emplace_back(xStart, xEnd);

            const CubicSpline::Node yStart{swingStartTime, currentFeetEndEffectors[j].y(), scaling * config_.liftOffVelocity};
            const CubicSpline::Node yEnd{swingFinalTime, targetFeetEndEffectors[j].front().y(), scaling * config_.touchDownVelocity};
            feetYTrajectories_[j].emplace_back(yStart, yEnd);
          }

      }
      else {  // for a stance leg
        int m = p;
        for(; m >= 0; m--){
          if(!eesContactFlagStocks[j][m]){
            break;
          }
        }
        if(p < initIndex){
          const CubicSpline::Node liftOff{0.0, currentFeetEndEffectors[j].z(), 0.0};
          const CubicSpline::Node touchDown{1.0, currentFeetEndEffectors[j].z(), 0.0};
          feetHeightTrajectories_[j].emplace_back(liftOff, currentFeetEndEffectors[j].z(), touchDown);

          const CubicSpline::Node xStart{0.0, currentFeetEndEffectors[j].x(), 0.0};
          const CubicSpline::Node xEnd{1.0, currentFeetEndEffectors[j].x(), 0.0};
          feetXTrajectories_[j].emplace_back(xStart, xEnd);

          const CubicSpline::Node yStart{0.0, currentFeetEndEffectors[j].y(), 0.0};
          const CubicSpline::Node yEnd{1.0, currentFeetEndEffectors[j].y(), 0.0};
          feetYTrajectories_[j].emplace_back(yStart, yEnd);
        }
        else if(m > initIndex){
          const CubicSpline::Node liftOff{0.0, targetFeetEndEffectors[j].front().z(), 0.0};
          const CubicSpline::Node touchDown{1.0, targetFeetEndEffectors[j].front().z(), 0.0};
          feetHeightTrajectories_[j].emplace_back(liftOff, targetFeetEndEffectors[j].front().z(), touchDown);

          const CubicSpline::Node xStart{0.0, targetFeetEndEffectors[j].front().x(), 0.0};
          const CubicSpline::Node xEnd{1.0, targetFeetEndEffectors[j].front().x(), 0.0};
          feetXTrajectories_[j].emplace_back(xStart, xEnd);

          const CubicSpline::Node yStart{0.0, targetFeetEndEffectors[j].front().y(), 0.0};
          const CubicSpline::Node yEnd{1.0, targetFeetEndEffectors[j].front().y(), 0.0};
          feetYTrajectories_[j].emplace_back(yStart, yEnd);
        }
        else{
          const CubicSpline::Node liftOff{0.0, targetFeetEndEffectors[j].back().z(), 0.0};
          const CubicSpline::Node touchDown{1.0, targetFeetEndEffectors[j].back().z(), 0.0};
          feetHeightTrajectories_[j].emplace_back(liftOff, targetFeetEndEffectors[j].back().z(), touchDown);

          const CubicSpline::Node xStart{0.0, targetFeetEndEffectors[j].back().x(), 0.0};
          const CubicSpline::Node xEnd{1.0, targetFeetEndEffectors[j].back().x(), 0.0};
          feetXTrajectories_[j].emplace_back(xStart, xEnd);

          const CubicSpline::Node yStart{0.0, targetFeetEndEffectors[j].back().y(), 0.0};
          const CubicSpline::Node yEnd{1.0, targetFeetEndEffectors[j].back().y(), 0.0};
          feetYTrajectories_[j].emplace_back(yStart, yEnd);
        }
      }
    }
    // if(j == 0){
    //     for(const auto& p:feetPlacement[j]){
    //       std::cout << "leg: " << j << " x: " << p.x() << " y: " << p.y() << " z: " << p.z() << std::endl;
    //     }
    // }
    
    feetHeightTrajectoriesEvents_[j] = eventTimes;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<std::vector<int>, std::vector<int>> SwingTrajectoryPlanner::updateFootSchedule(const std::vector<bool>& contactFlagStock) {
  const size_t numPhases = contactFlagStock.size();

  std::vector<int> startTimeIndexStock(numPhases, 0);
  std::vector<int> finalTimeIndexStock(numPhases, 0);

  // find the startTime and finalTime indices for swing feet
  for (size_t i = 0; i < numPhases; i++) {
    if (!contactFlagStock[i]) {
      std::tie(startTimeIndexStock[i], finalTimeIndexStock[i]) = findIndex(i, contactFlagStock);
    }
  }
  return {startTimeIndexStock, finalTimeIndexStock};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
feet_array_t<std::vector<bool>> SwingTrajectoryPlanner::extractContactFlags(const std::vector<size_t>& phaseIDsStock) const {
  const size_t numPhases = phaseIDsStock.size();

  feet_array_t<std::vector<bool>> contactFlagStock;
  std::fill(contactFlagStock.begin(), contactFlagStock.end(), std::vector<bool>(numPhases));

  for (size_t i = 0; i < numPhases; i++) {
    const auto contactFlag = modeNumber2StanceLeg(phaseIDsStock[i]);
    for (size_t j = 0; j < numFeet_; j++) {
      contactFlagStock[j][i] = contactFlag[j];
    }
  }
  return contactFlagStock;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<int, int> SwingTrajectoryPlanner::findIndex(size_t index, const std::vector<bool>& contactFlagStock) {
  const size_t numPhases = contactFlagStock.size();

  // skip if it is a stance leg
  if (contactFlagStock[index]) {
    return {0, 0};
  }

  // find the starting time
  int startTimesIndex = -1;
  for (int ip = index - 1; ip >= 0; ip--) {
    if (contactFlagStock[ip]) {
      startTimesIndex = ip;
      break;
    }
  }

  // find the final time
  int finalTimesIndex = numPhases - 1;
  for (size_t ip = index + 1; ip < numPhases; ip++) {
    if (contactFlagStock[ip]) {
      finalTimesIndex = ip - 1;
      break;
    }
  }

  return {startTimesIndex, finalTimesIndex};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex,
                                                      const std::vector<size_t>& phaseIDsStock) {
  const size_t numSubsystems = phaseIDsStock.size();
  if (startIndex < 0) {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++) {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of take-off for the first swing of the EE with ID " + std::to_string(leg) + " is not defined.");
  }
  if (finalIndex >= numSubsystems - 1) {
    std::cerr << "Subsystem: " << index << " out of " << numSubsystems - 1 << std::endl;
    for (size_t i = 0; i < numSubsystems; i++) {
      std::cerr << "[" << i << "]: " << phaseIDsStock[i] << ",  ";
    }
    std::cerr << std::endl;

    throw std::runtime_error("The time of touch-down for the last swing of the EE with ID " + std::to_string(leg) + " is not defined.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::swingTrajectoryScaling(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale) {
  return std::min(1.0, (finalTime - startTime) / swingTimeScale);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwingTrajectoryPlanner::Config loadSwingTrajectorySettings(const std::string& fileName, const std::string& fieldName, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);

  if (verbose) {
    std::cerr << "\n #### Swing Trajectory Config:";
    std::cerr << "\n #### =============================================================================\n";
  }

  SwingTrajectoryPlanner::Config config;
  const std::string prefix = fieldName + ".";

  loadData::loadPtreeValue(pt, config.liftOffVelocity, prefix + "liftOffVelocity", verbose);
  loadData::loadPtreeValue(pt, config.touchDownVelocity, prefix + "touchDownVelocity", verbose);
  loadData::loadPtreeValue(pt, config.swingHeight, prefix + "swingHeight", verbose);
  loadData::loadPtreeValue(pt, config.swingTimeScale, prefix + "swingTimeScale", verbose);

  if (verbose) {
    std::cerr << " #### =============================================================================" << std::endl;
  }

  return config;
}

}  // namespace legged_robot
}  // namespace ocs2
