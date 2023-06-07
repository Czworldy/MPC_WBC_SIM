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
SwingTrajectoryPlanner::SwingTrajectoryPlanner(Config config, size_t numFeet) : config_(std::move(config)), numFeet_(numFeet),
  minimumJerkSolver_(3) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getZvelocityConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  if(usingMultiHeight_){
    return feetMultiHeightTrajectories_[leg][index]->velocity(time);
  }
  else
    return feetHeightTrajectories_[leg][index].velocity(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
scalar_t SwingTrajectoryPlanner::getZpositionConstraint(size_t leg, scalar_t time) const {
  const auto index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[leg], time);
  if(usingMultiHeight_)
    return feetMultiHeightTrajectories_[leg][index]->position(time);
  else
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

// /******************************************************************************************************/
// /******************************************************************************************************/
// /******************************************************************************************************/
// void SwingTrajectoryPlanner::updateUsingMultiHeightAndSwingMiddleTime(const ModeSchedule& modeSchedule, 
//               const feet_array_t<std::vector<vector3_t>>& feetPlacement, scalar_t initTime,
//               const feet_array_t<vector3_t>& currentFeetEndEffectors,
//               const feet_array_t<std::vector<vector_t>>& swingHeightSequence, 
//               const feet_array_t<std::vector<scalar_t>>& swingMiddleTimeSequence) {
//   const auto& modeSequence = modeSchedule.modeSequence;
//   const auto& eventTimes = modeSchedule.eventTimes;

//   usingMultiHeight_ = true;

//   const auto eesContactFlagStocks = extractContactFlags(modeSequence);

//   const size_t initIndex = lookup::findIndexInTimeArray(eventTimes, initTime);


//   feet_array_t<std::vector<int>> startTimesIndices;
//   feet_array_t<std::vector<int>> finalTimesIndices;
//   for (size_t leg = 0; leg < numFeet_; leg++) {
//     std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
//   }

//   for (size_t j = 0; j < numFeet_; j++) {
//     if (eesContactFlagStocks[j][initIndex]){
//     // feetHeightTrajectories_[j].clear();
//     // feetHeightTrajectories_[j].reserve(modeSequence.size());

//     feetMultiHeightTrajectories_[j].clear();
//     feetMultiHeightTrajectories_[j].reserve(modeSequence.size());

//     feetXTrajectories_[j].clear();
//     feetXTrajectories_[j].reserve(modeSequence.size());

//     feetYTrajectories_[j].clear();
//     feetYTrajectories_[j].reserve(modeSequence.size());
//     for (int p = 0; p < modeSequence.size(); ++p) {
//       if (!eesContactFlagStocks[j][p]) {  // for a swing leg
//         // consider after swing phase another swing phase again.
//         int m = p;
//         for(; m > 0; m--){
//           if(eesContactFlagStocks[j][m]){
//             break;
//           }
//         }
//         const int swingStartIndex = startTimesIndices[j][p];
//         const int swingFinalIndex = finalTimesIndices[j][p];
//         checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

//         const scalar_t swingStartTime = eventTimes[swingStartIndex];
//         const scalar_t swingFinalTime = eventTimes[swingFinalIndex];

//         const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);

//         // scalar_t swingHeight = std::max(swingHeightSequence[j][p], 0.12);
//         // swingHeight = std::min(swingHeight, 0.3);

//         // if (p >= 1){
//           if (p == initIndex + 1) {
//             /* Node : T P V */
//             const scalar_t midHeight      = currentFeetEndEffectors[j].z() + scaling * swingHeightSequence[j][p][1];
//             const scalar_t midHeightLeft  = currentFeetEndEffectors[j].z() + scaling * swingHeightSequence[j][p][0];
//             const scalar_t midHeightRight = currentFeetEndEffectors[j].z() + scaling * swingHeightSequence[j][p][2];
//             const scalar_t midTime = swingStartTime + swingMiddleTimeSequence[j][p];
//             // const scalar_t midTime = (swingStartTime + swingFinalTime) / 2.;
//             const scalar_t swingTime = swingFinalTime - swingStartTime;

//             const CubicSpline::Node liftOff{swingStartTime, currentFeetEndEffectors[j].z(), scaling * config_.liftOffVelocity}; // without foothold from mapper, this with cause promblem in slope.
//             const CubicSpline::Node apex{midTime, midHeight, 0.0}; 
//             const CubicSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), config_.touchDownVelocity};

//             const SplineCpg leftSpline(liftOff, midHeightLeft, swingStartTime + swingMiddleTimeSequence[j][p]/2., 2*scaling * config_.liftOffVelocity/3., apex);
//             const SplineCpg rightSpline(apex, midHeightRight, (swingFinalTime + swingStartTime + swingMiddleTimeSequence[j][p])/2., scaling * config_.touchDownVelocity/3., touchDown);

//             // const SplineCpg leftSpline(liftOff, midHeightLeft, swingStartTime + 0.25 * swingTime, 2 * scaling * config_.liftOffVelocity/3., apex);
//             // const SplineCpg rightSpline(apex, midHeightRight,  swingStartTime + 0.75 * swingTime, scaling * config_.touchDownVelocity/3., touchDown);
              
//             feetMultiHeightTrajectories_[j].emplace_back(leftSpline, rightSpline, midTime);

//             std::cout << "midTime : " << j << " " << midTime << "\n";
//             std::cout << "midHeightLeft : " << j << " " << midHeightLeft << "\n";
//             std::cout << "midHeightRight : " << j << " " << midHeightRight << "\n";
//             std::cout << "midHeight : " << j << " " << midHeight << "\n";

//             const CubicSpline::Node xStart{swingStartTime, currentFeetEndEffectors[j].x(), 0.0};
//             const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0.0};
//             feetXTrajectories_[j].emplace_back(xStart, (xStart.position + xEnd.position)/2., 
//                                                midTime, scaling * config_.liftOffVelocity, xEnd);

//             const CubicSpline::Node yStart{swingStartTime, currentFeetEndEffectors[j].y(), 0.0};
//             const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0.};
//             feetYTrajectories_[j].emplace_back(yStart, (yStart.position + yEnd.position)/2., 
//                                                midTime, scaling * config_.liftOffVelocity, yEnd);
//           }
//           else{
//             const scalar_t midHeight      = feetPlacement[j][m].z() + scaling * swingHeightSequence[j][p][1];
//             const scalar_t midHeightLeft  = feetPlacement[j][m].z() + scaling * swingHeightSequence[j][p][0];
//             const scalar_t midHeightRight = feetPlacement[j][m].z() + scaling * swingHeightSequence[j][p][2];
//             const scalar_t midTime = swingStartTime + swingMiddleTimeSequence[j][p];
//             // const scalar_t midTime = (swingStartTime + swingFinalTime) / 2.;
//             const scalar_t swingTime = swingFinalTime - swingStartTime;

//             const CubicSpline::Node liftOff{swingStartTime, feetPlacement[j][m].z(), scaling * config_.liftOffVelocity}; // without foothold from mapper, this with cause promblem in slope.
//             const CubicSpline::Node apex{midTime, midHeight, 0.0}; 
//             const CubicSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), config_.touchDownVelocity};

//             const SplineCpg leftSpline(liftOff, midHeightLeft, swingStartTime + swingMiddleTimeSequence[j][p]/2., 2*scaling * config_.liftOffVelocity/3., apex);
//             const SplineCpg rightSpline(apex, midHeightRight, (swingFinalTime + swingStartTime + swingMiddleTimeSequence[j][p])/2., scaling * config_.touchDownVelocity/3., touchDown);
            
//             // const SplineCpg leftSpline(liftOff, midHeightLeft, swingStartTime + 0.25 * swingTime, 2 * scaling * config_.liftOffVelocity/3., apex);
//             // const SplineCpg rightSpline(apex, midHeightRight,  swingStartTime + 0.75 * swingTime, scaling * config_.touchDownVelocity/3., touchDown);
//             feetMultiHeightTrajectories_[j].emplace_back(leftSpline, rightSpline, midTime);

//             std::cout << "midTime : " << j << " " << midTime << "\n";
//             std::cout << "midHeightLeft : " << j << " " << midHeightLeft << "\n";
//             std::cout << "midHeightRight : " << j << " " << midHeightRight << "\n";
//             std::cout << "midTimeLeft : " << j << " " << swingStartTime + swingMiddleTimeSequence[j][p]/2. << "\n";
//             std::cout << "midTimeRight : " << j << " " << (swingFinalTime + swingStartTime + swingMiddleTimeSequence[j][p])/2. << "\n";
//             std::cout << "midHeight : " << j << " " << midHeight << "\n";

//             const CubicSpline::Node xStart{swingStartTime, feetPlacement[j][m].x(), 0.0};
//             const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0.0};
//             feetXTrajectories_[j].emplace_back(xStart, (xStart.position + xEnd.position)/2., 
//                                                midTime, scaling * config_.liftOffVelocity, xEnd);

//             const CubicSpline::Node yStart{swingStartTime, feetPlacement[j][m].y(), 0.0};
//             const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0.};
//             feetYTrajectories_[j].emplace_back(yStart, (yStart.position + yEnd.position)/2., 
//                                                midTime, scaling * config_.liftOffVelocity, yEnd);
//           }
//       } else {  // for a stance leg
//         const CubicSpline::Node liftOff{0.0, feetPlacement[j][p].z(), 0.0};
//         const CubicSpline::Node apex{0.5, feetPlacement[j][p].z(), 0.0};
//         const CubicSpline::Node touchDown{1.0, feetPlacement[j][p].z(), 0.0};
//         const SplineCpg leftSpline(liftOff, feetPlacement[j][p].z(), 0.25, 0.0, apex);
//         const SplineCpg rightSpline(apex, feetPlacement[j][p].z(), 0.75, 0.0, touchDown);
        
//         feetMultiHeightTrajectories_[j].emplace_back(leftSpline, rightSpline, 0.5);

//         const CubicSpline::Node xStart{0.0, feetPlacement[j][p].x(), 0.0};
//         const CubicSpline::Node xEnd{1.0, feetPlacement[j][p].x(), 0.0};
//         feetXTrajectories_[j].emplace_back(xStart, feetPlacement[j][p].x(),xEnd);

//         const CubicSpline::Node yStart{0.0, feetPlacement[j][p].y(), 0.0};
//         const CubicSpline::Node yEnd{1.0, feetPlacement[j][p].y(), 0.0};
//         feetYTrajectories_[j].emplace_back(yStart, feetPlacement[j][p].y(),yEnd);
//       }
//     }
//     // if(j == 0){
//     //     for(const auto& p:feetPlacement[j]){
//     //       std::cout << "leg: " << j << " x: " << p.x() << " y: " << p.y() << " z: " << p.z() << std::endl;
//     //     }
//     // }
    
//     feetHeightTrajectoriesEvents_[j] = eventTimes;
//   }
//   }
// } 

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// void SwingTrajectoryPlanner::updateUsingMultiHeightAndSwingMiddleTime(const ModeSchedule& modeSchedule, 
//               const feet_array_t<std::vector<vector3_t>>& feetPlacement, scalar_t initTime,
//               const feet_array_t<vector3_t>& currentFeetEndEffectors,
//               const feet_array_t<std::vector<vector_t>>& swingHeightSequence, 
//               const feet_array_t<std::vector<scalar_t>>& swingMiddleTimeSequence) {
//   const auto& modeSequence = modeSchedule.modeSequence;
//   const auto& eventTimes = modeSchedule.eventTimes;

//   usingMultiHeight_ = true;

//   const auto eesContactFlagStocks = extractContactFlags(modeSequence);

//   const size_t initIndex = lookup::findIndexInTimeArray(eventTimes, initTime);


//   feet_array_t<std::vector<int>> startTimesIndices;
//   feet_array_t<std::vector<int>> finalTimesIndices;
//   for (size_t leg = 0; leg < numFeet_; leg++) {
//     std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
//   }

//   for (size_t j = 0; j < numFeet_; j++) {
//     if (eesContactFlagStocks[j][initIndex]){ // current stance leg
//     // feetHeightTrajectories_[j].clear();
//     // feetHeightTrajectories_[j].reserve(modeSequence.size());

//     feetMultiHeightTrajectories_[j].clear();
//     feetMultiHeightTrajectories_[j].reserve(modeSequence.size());

//     feetXTrajectories_[j].clear();
//     feetXTrajectories_[j].reserve(modeSequence.size());

//     feetYTrajectories_[j].clear();
//     feetYTrajectories_[j].reserve(modeSequence.size());
//     for (int p = 0; p < modeSequence.size(); ++p) {
//       if (!eesContactFlagStocks[j][p]) {  // for a swing leg
//         // consider after swing phase another swing phase again.
//         int m = p;
//         for(; m > 0; m--){
//           if(eesContactFlagStocks[j][m]){
//             break;
//           }
//         }
//         const int swingStartIndex = startTimesIndices[j][p];
//         const int swingFinalIndex = finalTimesIndices[j][p];
//         checkThatIndicesAreValid(j, p, swingStartIndex, swingFinalIndex, modeSequence);

//         const scalar_t swingStartTime = eventTimes[swingStartIndex];
//         const scalar_t swingFinalTime = eventTimes[swingFinalIndex];

//         const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);

//         // scalar_t swingHeight = std::max(swingHeightSequence[j][p], 0.12);
//         // swingHeight = std::min(swingHeight, 0.3);

//         // if (p >= 1){
//           if (p == initIndex + 1) {
//             /* Node : T P V */
//             const scalar_t midHeight      = currentFeetEndEffectors[j].z() + scaling * swingHeightSequence[j][p][1];
//             const scalar_t midHeightLeft  = currentFeetEndEffectors[j].z() + scaling * swingHeightSequence[j][p][0];
//             const scalar_t midHeightRight = currentFeetEndEffectors[j].z() + scaling * swingHeightSequence[j][p][2];
//             const scalar_t midTime = swingStartTime + swingMiddleTimeSequence[j][p];
//             // const scalar_t midTime = (swingStartTime + swingFinalTime) / 2.;
//             const scalar_t swingTime = swingFinalTime - swingStartTime;

//             const CubicSpline::Node liftOff{swingStartTime, currentFeetEndEffectors[j].z(), scaling * config_.liftOffVelocity}; // without foothold from mapper, this with cause promblem in slope.
//             const CubicSpline::Node apex{midTime, midHeight, 0.0}; 
//             const CubicSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), scaling * config_.touchDownVelocity};

//             const SplineCpg leftSpline(liftOff, midHeightLeft, swingStartTime + swingMiddleTimeSequence[j][p]/2., scaling * config_.liftOffVelocity, apex);
//             const SplineCpg rightSpline(apex, midHeightRight, (swingFinalTime + swingStartTime + swingMiddleTimeSequence[j][p])/2., scaling * config_.touchDownVelocity, touchDown);

//             // const SplineCpg leftSpline(liftOff, midHeightLeft, swingStartTime + 0.25 * swingTime, 2 * scaling * config_.liftOffVelocity/3., apex);
//             // const SplineCpg rightSpline(apex, midHeightRight,  swingStartTime + 0.75 * swingTime, scaling * config_.touchDownVelocity/3., touchDown);
              
//             feetMultiHeightTrajectories_[j].emplace_back(leftSpline, rightSpline, midTime);

//             // std::cout << "midTime : " << j << " " << midTime << "\n";
//             // std::cout << "midHeightLeft : " << j << " " << midHeightLeft << "\n";
//             // std::cout << "midHeightRight : " << j << " " << midHeightRight << "\n";
//             // std::cout << "midHeight : " << j << " " << midHeight << "\n";

//             const CubicSpline::Node xStart{swingStartTime, currentFeetEndEffectors[j].x(), 0.0};
//             const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0.0};
//             feetXTrajectories_[j].emplace_back(xStart, (xStart.position + xEnd.position)/2., 
//                                                midTime, scaling * config_.liftOffVelocity, xEnd);

//             const CubicSpline::Node yStart{swingStartTime, currentFeetEndEffectors[j].y(), 0.0};
//             const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0.};
//             feetYTrajectories_[j].emplace_back(yStart, (yStart.position + yEnd.position)/2., 
//                                                midTime, scaling * config_.liftOffVelocity, yEnd);
//           }
//           else{
//             const scalar_t midHeight      = feetPlacement[j][m].z() + scaling * swingHeightSequence[j][p][1];
//             const scalar_t midHeightLeft  = feetPlacement[j][m].z() + scaling * swingHeightSequence[j][p][0];
//             const scalar_t midHeightRight = feetPlacement[j][m].z() + scaling * swingHeightSequence[j][p][2];
//             const scalar_t midTime = swingStartTime + swingMiddleTimeSequence[j][p];
//             // const scalar_t midTime = (swingStartTime + swingFinalTime) / 2.;
//             const scalar_t swingTime = swingFinalTime - swingStartTime;

//             const CubicSpline::Node liftOff{swingStartTime, feetPlacement[j][m].z(), scaling * config_.liftOffVelocity}; // without foothold from mapper, this with cause promblem in slope.
//             const CubicSpline::Node apex{midTime, midHeight, 0.0}; 
//             const CubicSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), scaling * config_.touchDownVelocity};

//             const SplineCpg leftSpline(liftOff, midHeightLeft, swingStartTime + swingMiddleTimeSequence[j][p]/2., scaling * config_.liftOffVelocity, apex);
//             const SplineCpg rightSpline(apex, midHeightRight, (swingFinalTime + swingStartTime + swingMiddleTimeSequence[j][p])/2., scaling * config_.touchDownVelocity, touchDown);
            
//             // const SplineCpg leftSpline(liftOff, midHeightLeft, swingStartTime + 0.25 * swingTime, 2 * scaling * config_.liftOffVelocity/3., apex);
//             // const SplineCpg rightSpline(apex, midHeightRight,  swingStartTime + 0.75 * swingTime, scaling * config_.touchDownVelocity/3., touchDown);
//             feetMultiHeightTrajectories_[j].emplace_back(leftSpline, rightSpline, midTime);

//             // std::cout << "midTime : " << j << " " << midTime << "\n";
//             // std::cout << "midHeightLeft : " << j << " " << midHeightLeft << "\n";
//             // std::cout << "midHeightRight : " << j << " " << midHeightRight << "\n";
//             // std::cout << "midTimeLeft : " << j << " " << swingStartTime + swingMiddleTimeSequence[j][p]/2. << "\n";
//             // std::cout << "midTimeRight : " << j << " " << (swingFinalTime + swingStartTime + swingMiddleTimeSequence[j][p])/2. << "\n";
//             // std::cout << "midHeight : " << j << " " << midHeight << "\n";

//             const CubicSpline::Node xStart{swingStartTime, feetPlacement[j][m].x(), 0.0};
//             const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0.0};
//             feetXTrajectories_[j].emplace_back(xStart, (xStart.position + xEnd.position)/2., 
//                                                midTime, scaling * config_.liftOffVelocity, xEnd);

//             const CubicSpline::Node yStart{swingStartTime, feetPlacement[j][m].y(), 0.0};
//             const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0.};
//             feetYTrajectories_[j].emplace_back(yStart, (yStart.position + yEnd.position)/2., 
//                                                midTime, scaling * config_.liftOffVelocity, yEnd);
//           }
//       } else {  // for a stance leg
//         const CubicSpline::Node liftOff{0.0, feetPlacement[j][p].z(), 0.0};
//         const CubicSpline::Node apex{0.5, feetPlacement[j][p].z(), 0.0};
//         const CubicSpline::Node touchDown{1.0, feetPlacement[j][p].z(), 0.0};
//         const SplineCpg leftSpline(liftOff, feetPlacement[j][p].z(), 0.25, 0.0, apex);
//         const SplineCpg rightSpline(apex, feetPlacement[j][p].z(), 0.75, 0.0, touchDown);
        
//         feetMultiHeightTrajectories_[j].emplace_back(leftSpline, rightSpline, 0.5);

//         const CubicSpline::Node xStart{0.0, feetPlacement[j][p].x(), 0.0};
//         const CubicSpline::Node xEnd{1.0, feetPlacement[j][p].x(), 0.0};
//         feetXTrajectories_[j].emplace_back(xStart, feetPlacement[j][p].x(),xEnd);

//         const CubicSpline::Node yStart{0.0, feetPlacement[j][p].y(), 0.0};
//         const CubicSpline::Node yEnd{1.0, feetPlacement[j][p].y(), 0.0};
//         feetYTrajectories_[j].emplace_back(yStart, feetPlacement[j][p].y(),yEnd);
//       }
//     }
//     // if(j == 0){
//     //     for(const auto& p:feetPlacement[j]){
//     //       std::cout << "leg: " << j << " x: " << p.x() << " y: " << p.y() << " z: " << p.z() << std::endl;
//     //     }
//     // }
    
//     feetHeightTrajectoriesEvents_[j] = eventTimes;
//   }
//   else {
//     //copy the previous leg placement according to the current event time.
//     std::vector<MultiSplineCpg> feetMultiHeightTrajectoriesTemp;
//     std::vector<SplineCpg> feetXTrajectoriesTemp;
//     std::vector<SplineCpg> feetYTrajectoriesTemp;
//     for (int p = 0; p < eventTimes.size(); ++p) {
//         size_t index = lookup::findIndexInTimeArray(feetHeightTrajectoriesEvents_[j], eventTimes[p]);
//         feetMultiHeightTrajectoriesTemp.emplace_back(feetMultiHeightTrajectories_[j][index]);
//         feetXTrajectoriesTemp.emplace_back(feetXTrajectories_[j][index]);
//         feetYTrajectoriesTemp.emplace_back(feetYTrajectories_[j][index]);
//     }
//     feetMultiHeightTrajectories_[j] = feetMultiHeightTrajectoriesTemp; 
//     feetXTrajectories_[j] = feetXTrajectoriesTemp;
//     feetYTrajectories_[j] = feetYTrajectoriesTemp;
//     feetHeightTrajectoriesEvents_[j] = eventTimes;
//   }
//   }
// } 

void SwingTrajectoryPlanner::update(const ModeSchedule& modeSchedule, const feet_array_t<std::vector<vector3_t>>& feetPlacement, 
                                    scalar_t initTime, const feet_array_t<vector3_t>& currentFeetEndEffectors, bool isLateTouchdown) {
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
    if (eesContactFlagStocks[j][initIndex] || true){
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

        const scalar_t midTime = (swingStartTime + swingFinalTime) / 2.;

        const scalar_t scaling = swingTrajectoryScaling(swingStartTime, swingFinalTime, config_.swingTimeScale);

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
          if (p == initIndex + 1) {
            /* code */
            // std::cout << "Late touchdown in phase: " << initIndex << std::endl;
            const CubicSpline::Node liftOff{swingStartTime, currentFeetEndEffectors[j].z(), scaling * config_.liftOffVelocity};
            const CubicSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), -1.0};
            const scalar_t midHeight = std::min(currentFeetEndEffectors[j].z(), feetPlacement[j][p].z()) + scaling * config_.swingHeight;
            feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);

            // const CubicSpline::Node xStart{swingStartTime, currentFeetEndEffectors[j].x(), 0};
            // const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0};
            // feetXTrajectories_[j].emplace_back(xStart, xEnd);

            // const CubicSpline::Node yStart{swingStartTime, currentFeetEndEffectors[j].y(), 0};
            // const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0};
            // feetYTrajectories_[j].emplace_back(yStart, yEnd);

            const CubicSpline::Node xStart{swingStartTime, currentFeetEndEffectors[j].x(), 0.0};
            const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0.0};
            feetXTrajectories_[j].emplace_back(xStart, (xStart.position + xEnd.position)/2., 
                                               midTime, scaling * config_.liftOffVelocity, xEnd);

            const CubicSpline::Node yStart{swingStartTime, currentFeetEndEffectors[j].y(), 0.0};
            const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0.};
            feetYTrajectories_[j].emplace_back(yStart, (yStart.position + yEnd.position)/2., 
                                               midTime, scaling * config_.liftOffVelocity, yEnd);
          }
          else{
          const CubicSpline::Node liftOff{swingStartTime, feetPlacement[j][m].z(), scaling * config_.liftOffVelocity};
          const CubicSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), scaling * config_.touchDownVelocity};
          const scalar_t midHeight = std::min(feetPlacement[j][p-1].z(), feetPlacement[j][p].z()) + scaling * config_.swingHeight;
          feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);

          // const CubicSpline::Node xStart{swingStartTime, feetPlacement[j][m].x(), scaling * config_.liftOffVelocity};
          // const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), scaling * 0};
          // feetXTrajectories_[j].emplace_back(xStart, xEnd);

          // const CubicSpline::Node yStart{swingStartTime, feetPlacement[j][m].y(), scaling * config_.liftOffVelocity};
          // const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), scaling * 0};
          // feetYTrajectories_[j].emplace_back(yStart, yEnd);
          const CubicSpline::Node xStart{swingStartTime, feetPlacement[j][m].x(), 0.0};
          const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0.0};
          feetXTrajectories_[j].emplace_back(xStart, (xStart.position + xEnd.position)/2., 
                                              midTime, scaling * config_.liftOffVelocity, xEnd);

          const CubicSpline::Node yStart{swingStartTime, feetPlacement[j][m].y(), 0.0};
          const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0.};
          feetYTrajectories_[j].emplace_back(yStart, (yStart.position + yEnd.position)/2., 
                                              midTime, scaling * config_.liftOffVelocity, yEnd);
          }
        }
        else{
          const CubicSpline::Node liftOff{swingStartTime, feetPlacement[j][p].z(), scaling * config_.liftOffVelocity};
          const CubicSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), scaling * config_.touchDownVelocity};
          const scalar_t midHeight = std::min(feetPlacement[j][p-1].z(), feetPlacement[j][p].z()) + scaling * config_.swingHeight;
          feetHeightTrajectories_[j].emplace_back(liftOff, midHeight, touchDown);

          // const CubicSpline::Node xStart{swingStartTime, feetPlacement[j][p].x(), scaling * config_.liftOffVelocity};
          // const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), scaling * 0};
          // feetXTrajectories_[j].emplace_back(xStart, xEnd);

          // const CubicSpline::Node yStart{swingStartTime, feetPlacement[j][p].y(), scaling * config_.liftOffVelocity};
          // const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), scaling * 0};
          // feetYTrajectories_[j].emplace_back(yStart, yEnd);
          const CubicSpline::Node xStart{swingStartTime, feetPlacement[j][m].x(), 0.0};
          const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0.0};
          feetXTrajectories_[j].emplace_back(xStart, (xStart.position + xEnd.position)/2., 
                                              midTime, scaling * config_.liftOffVelocity, xEnd);

          const CubicSpline::Node yStart{swingStartTime, feetPlacement[j][m].y(), 0.0};
          const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0.};
          feetYTrajectories_[j].emplace_back(yStart, (yStart.position + yEnd.position)/2., 
                                              midTime, scaling * config_.liftOffVelocity, yEnd);
        }
        
      } else {  // for a stance leg
        const CubicSpline::Node liftOff{0.0, feetPlacement[j][p].z(), 0.0};
        const CubicSpline::Node touchDown{1.0, feetPlacement[j][p].z(), 0.0};
        feetHeightTrajectories_[j].emplace_back(liftOff, feetPlacement[j][p].z(), touchDown);

        const CubicSpline::Node xStart{0.0, feetPlacement[j][p].x(), 0.0};
        const CubicSpline::Node xEnd{1.0, feetPlacement[j][p].x(), 0.0};
        feetXTrajectories_[j].emplace_back(xStart, feetPlacement[j][p].x(),xEnd);

        const CubicSpline::Node yStart{0.0, feetPlacement[j][p].y(), 0.0};
        const CubicSpline::Node yEnd{1.0, feetPlacement[j][p].y(), 0.0};
        feetYTrajectories_[j].emplace_back(yStart, feetPlacement[j][p].y(),yEnd);
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

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwingTrajectoryPlanner::updateUsingMultiHeightAndSwingMiddleTime(const ModeSchedule& modeSchedule, 
              const feet_array_t<std::vector<vector3_t>>& feetPlacement, scalar_t initTime,
              const feet_array_t<vector3_t>& currentFeetEndEffectors,
              const feet_array_t<std::vector<vector_t>>& swingHeightSequence, 
              const feet_array_t<std::vector<scalar_t>>& swingMiddleTimeSequence) {
  const auto& modeSequence = modeSchedule.modeSequence;
  const auto& eventTimes = modeSchedule.eventTimes;

  usingMultiHeight_ = true;

  const auto eesContactFlagStocks = extractContactFlags(modeSequence);

  const size_t initIndex = lookup::findIndexInTimeArray(eventTimes, initTime);


  feet_array_t<std::vector<int>> startTimesIndices;
  feet_array_t<std::vector<int>> finalTimesIndices;
  for (size_t leg = 0; leg < numFeet_; leg++) {
    std::tie(startTimesIndices[leg], finalTimesIndices[leg]) = updateFootSchedule(eesContactFlagStocks[leg]);
  }

  for (size_t j = 0; j < numFeet_; j++) {
    if (eesContactFlagStocks[j][initIndex] || true){
    // feetHeightTrajectories_[j].clear();
    // feetHeightTrajectories_[j].reserve(modeSequence.size());

    feetMultiHeightTrajectories_[j].clear();
    feetMultiHeightTrajectories_[j].reserve(modeSequence.size());

    feetXTrajectories_[j].clear();
    feetXTrajectories_[j].reserve(modeSequence.size());

    feetYTrajectories_[j].clear();
    feetYTrajectories_[j].reserve(modeSequence.size());
    for (int p = 0; p < modeSequence.size(); ++p) {
      if (!eesContactFlagStocks[j][p]) {  // for a swing leg
        // consider after swing phase another swing phase again.
        int m = p;
        for(; m > 0; m--){
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

        // scalar_t swingHeight = std::max(swingHeightSequence[j][p], 0.12);
        // swingHeight = std::min(swingHeight, 0.3);

        // if (p >= 1){
          if (p == initIndex + 1) {
            /* Node : T P V */
            const scalar_t midHeight      = currentFeetEndEffectors[j].z() + scaling * swingHeightSequence[j][p][1];
            const scalar_t midHeightLeft  = currentFeetEndEffectors[j].z() + scaling * swingHeightSequence[j][p][0];
            const scalar_t midHeightRight = currentFeetEndEffectors[j].z() + scaling * swingHeightSequence[j][p][2];
            const scalar_t midTime = swingStartTime + swingMiddleTimeSequence[j][p];
            // const scalar_t midTime = (swingStartTime + swingFinalTime) / 2.;
            const scalar_t swingTime = swingFinalTime - swingStartTime;

            const QuinticSpline::Node liftOff{swingStartTime, currentFeetEndEffectors[j].z(), scaling * config_.liftOffVelocity, 0.5}; // without foothold from mapper, this with cause promblem in slope.
            const QuinticSpline::Node middleLeft{swingStartTime + swingMiddleTimeSequence[j][p]/2.0, midHeightLeft, scaling * config_.liftOffVelocity, 0.0}; 
            const QuinticSpline::Node middleRight{(swingFinalTime + swingStartTime + swingMiddleTimeSequence[j][p])/2.0, midHeightLeft, scaling * config_.touchDownVelocity, 0.0}; 
            const QuinticSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), config_.touchDownVelocity, 0.0};

            const QuinticSpline::Node apex{midTime, midHeight, 0.0, 0.0};

            // feetMultiHeightTrajectories_[j].emplace_back(liftOff, middleLeft, midHeight, midTime, middleRight, touchDown);
            // feetMultiHeightTrajectories_[j].emplace_back(liftOff, midHeightLeft, apex, midHeightRight, touchDown);
            // std::unique_ptr<TwoSixthOrderSplineCpg> splinePtr(new TwoSixthOrderSplineCpg(liftOff, midHeightLeft, apex, midHeightRight, touchDown));
            auto coffe = minimumJerkSolver_.solveCoffectient(liftOff, midHeightLeft, apex, midHeightRight, touchDown);
            SenvenOrderSpline leftSpline(coffe.head(8), swingStartTime, midTime);
            SenvenOrderSpline rightSpline(coffe.tail(8), midTime, swingFinalTime);
            std::unique_ptr<SeventhOrderSplineCpg> splinePtr(new SeventhOrderSplineCpg(leftSpline, rightSpline, midTime));
            feetMultiHeightTrajectories_[j].emplace_back(std::move(splinePtr));
            
            // std::cout << "midTime : " << j << " " << midTime << "\n";
            // std::cout << "midHeightLeft : " << j << " " << midHeightLeft << "\n";
            // std::cout << "midHeightRight : " << j << " " << midHeightRight << "\n";
            // std::cout << "midHeight : " << j << " " << midHeight << "\n";

            const CubicSpline::Node xStart{swingStartTime, currentFeetEndEffectors[j].x(), 0.0};
            const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0.0};
            feetXTrajectories_[j].emplace_back(xStart, (xStart.position + xEnd.position)/2., 
                                               midTime, 2*scaling * config_.liftOffVelocity/3, xEnd);

            const CubicSpline::Node yStart{swingStartTime, currentFeetEndEffectors[j].y(), 0.0};
            const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0.};
            feetYTrajectories_[j].emplace_back(yStart, (yStart.position + yEnd.position)/2., 
                                               midTime, 2*scaling * config_.liftOffVelocity/3, yEnd);

            //CubeSpline
            // const CubicSpline::Node xStart{swingStartTime, currentFeetEndEffectors[j].x(), scaling * config_.liftOffVelocity};
            // const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0.0};
            // feetXTrajectories_[j].emplace_back(xStart, xEnd);

            // const CubicSpline::Node yStart{swingStartTime, currentFeetEndEffectors[j].y(), scaling * config_.liftOffVelocity};
            // const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0.};
            // feetYTrajectories_[j].emplace_back(yStart, yEnd);
          }
          else{
            const scalar_t midHeight      = feetPlacement[j][m].z() + scaling * swingHeightSequence[j][p][1];
            const scalar_t midHeightLeft  = feetPlacement[j][m].z() + scaling * swingHeightSequence[j][p][0];
            const scalar_t midHeightRight = feetPlacement[j][m].z() + scaling * swingHeightSequence[j][p][2];
            const scalar_t midTime = swingStartTime + swingMiddleTimeSequence[j][p];
            // const scalar_t midTime = (swingStartTime + swingFinalTime) / 2.;
            const scalar_t swingTime = swingFinalTime - swingStartTime;
            
            const QuinticSpline::Node liftOff{swingStartTime, feetPlacement[j][m].z(), scaling * config_.liftOffVelocity, 0.5}; // without foothold from mapper, this with cause promblem in slope.
            const QuinticSpline::Node middleLeft{swingStartTime + swingMiddleTimeSequence[j][p]/2.0, midHeightLeft, scaling * config_.liftOffVelocity, 0.0}; 
            const QuinticSpline::Node middleRight{(swingFinalTime + swingStartTime + swingMiddleTimeSequence[j][p])/2.0, midHeightLeft, scaling * config_.touchDownVelocity, 0.0}; 
            const QuinticSpline::Node touchDown{swingFinalTime, feetPlacement[j][p].z(), config_.touchDownVelocity, 0.0};

            const QuinticSpline::Node apex{midTime, midHeight, 0.0, 0.0};

            // feetMultiHeightTrajectories_[j].emplace_back(liftOff, middleLeft, midHeight, midTime, middleRight, touchDown);
            // feetMultiHeightTrajectories_[j].emplace_back(liftOff, midHeightLeft, apex, midHeightRight, touchDown);
            // std::unique_ptr<TwoSixthOrderSplineCpg> splinePtr(new TwoSixthOrderSplineCpg(liftOff, midHeightLeft, apex, midHeightRight, touchDown));
            auto coffe = minimumJerkSolver_.solveCoffectient(liftOff, midHeightLeft, apex, midHeightRight, touchDown);
            SenvenOrderSpline leftSpline(coffe.head(8), swingStartTime, midTime);
            SenvenOrderSpline rightSpline(coffe.tail(8), midTime, swingFinalTime);
            std::unique_ptr<SeventhOrderSplineCpg> splinePtr(new SeventhOrderSplineCpg(leftSpline, rightSpline, midTime));
            feetMultiHeightTrajectories_[j].emplace_back(std::move(splinePtr));

            // std::cout << "midTime : " << j << " " << midTime << "\n";
            // std::cout << "midHeightLeft : " << j << " " << midHeightLeft << "\n";
            // std::cout << "midHeightRight : " << j << " " << midHeightRight << "\n";
            // std::cout << "midTimeLeft : " << j << " " << swingStartTime + swingMiddleTimeSequence[j][p]/2. << "\n";
            // std::cout << "midTimeRight : " << j << " " << (swingFinalTime + swingStartTime + swingMiddleTimeSequence[j][p])/2. << "\n";
            // std::cout << "midHeight : " << j << " " << midHeight << "\n";


            const CubicSpline::Node xStart{swingStartTime, feetPlacement[j][m].x(), 0.0};
            const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0.0};
            feetXTrajectories_[j].emplace_back(xStart, (xStart.position + xEnd.position)/2., 
                                               midTime, 2*scaling * config_.liftOffVelocity / 3, xEnd);

            const CubicSpline::Node yStart{swingStartTime, feetPlacement[j][m].y(), 0.0};
            const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0.0};
            feetYTrajectories_[j].emplace_back(yStart, (yStart.position + yEnd.position)/2., 
                                               midTime, 2*scaling * config_.liftOffVelocity/3, yEnd);

            //CubeSpline
            // const CubicSpline::Node xStart{swingStartTime, currentFeetEndEffectors[j].x(), scaling * config_.liftOffVelocity};
            // const CubicSpline::Node xEnd{swingFinalTime, feetPlacement[j][p].x(), 0.0};
            // feetXTrajectories_[j].emplace_back(xStart, xEnd);

            // const CubicSpline::Node yStart{swingStartTime, currentFeetEndEffectors[j].y(), scaling * config_.liftOffVelocity};
            // const CubicSpline::Node yEnd{swingFinalTime, feetPlacement[j][p].y(), 0.};
            // feetYTrajectories_[j].emplace_back(yStart, yEnd);
          }
      } else {  // for a stance leg
        const QuinticSpline::Node liftOff{0.0, feetPlacement[j][p].z(), 0.0, 0.0};
        const QuinticSpline::Node middleLeft{0.25, feetPlacement[j][p].z(), 0.0, 0.0}; 
        const QuinticSpline::Node middleRight{0.75, feetPlacement[j][p].z(), 0.0, 0.0}; 
        const QuinticSpline::Node touchDown{1.0, feetPlacement[j][p].z(), 0.0, 0.0};
        const QuinticSpline::Node apex{0.5, feetPlacement[j][p].z(), 0.0, 0.0};

        // feetMultiHeightTrajectories_[j].emplace_back(liftOff, middleLeft, feetPlacement[j][p].z(), 0.5, middleRight, touchDown);
        // feetMultiHeightTrajectories_[j].emplace_back(liftOff, feetPlacement[j][p].z(), apex, feetPlacement[j][p].z(), touchDown);
        std::unique_ptr<TwoSixthOrderSplineCpg> splinePtr(new TwoSixthOrderSplineCpg(liftOff, feetPlacement[j][p].z(), apex, feetPlacement[j][p].z(), touchDown));
        feetMultiHeightTrajectories_[j].emplace_back(std::move(splinePtr));
        
        const CubicSpline::Node xStart{0.0, feetPlacement[j][p].x(), 0.0};
        const CubicSpline::Node xEnd{1.0, feetPlacement[j][p].x(), 0.0};
        feetXTrajectories_[j].emplace_back(xStart, feetPlacement[j][p].x(), xEnd);
        // feetXTrajectories_[j].emplace_back(xStart, xEnd);

        const CubicSpline::Node yStart{0.0, feetPlacement[j][p].y(), 0.0};
        const CubicSpline::Node yEnd{1.0, feetPlacement[j][p].y(), 0.0};
        feetYTrajectories_[j].emplace_back(yStart, feetPlacement[j][p].y(), yEnd);
        // feetYTrajectories_[j].emplace_back(yStart, yEnd);
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
