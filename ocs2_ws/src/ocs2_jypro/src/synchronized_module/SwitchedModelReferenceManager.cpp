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
#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/centroidal-derivatives.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include "ocs2_jypro/synchronized_module/SwitchedModelReferenceManager.h"
// #include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr,
                                                             std::shared_ptr<FootConstraintsPlanner> footPlacementPlannerPtr,
                                                             std::shared_ptr<LeggedIKSolver> LeggedIKSolverPtr,
                                                             const CentroidalModelPinocchioMapping& mapping,
                                                             PinocchioInterface& pinocchioInterface,
                                                             const CentroidalModelInfo& centroidalModelInfo,
                                                             std::shared_ptr<TerrainEstData> terrainEstDataPtr,
                                                             std::shared_ptr<feet_polygon_array_t> mpcPolygonArrayPtr,
                                                             std::shared_ptr<feet_array_t<std::vector<vector3_t>>> mpcNominalFeetholdsPtr,
                                                             std::shared_ptr<feet_array_t<std::vector<vector_t>>> mpcSwingHeightPtr,
                                                             std::shared_ptr<feet_array_t<std::vector<scalar_t>>> mpcSwingMiddleTimePtr)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)),
      footPlacementPlannerPtr_(std::move(footPlacementPlannerPtr)),
      LeggedIKSolverPtr_(std::move(LeggedIKSolverPtr)),
      mappingPtr_(mapping.clone()),
      pinocchioInterface_(pinocchioInterface),
      centroidalModelInfo_(centroidalModelInfo),
      terrainEstDataPtr_(std::move(terrainEstDataPtr)),
      mpcPolygonArrayPtr_(std::move(mpcPolygonArrayPtr)),
      mpcNominalFeetholdsPtr_(std::move(mpcNominalFeetholdsPtr)),
      mpcSwingHeightPtr_(std::move(mpcSwingHeightPtr)),
      mpcSwingMiddleTimePtr_(std::move(mpcSwingMiddleTimePtr)) { mappingPtr_->setPinocchioInterface(pinocchioInterface_); }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
contact_flag_t SwitchedModelReferenceManager::getContactFlags(scalar_t time) const {
  return modeNumber2StanceLeg(this->getModeSchedule().modeAtTime(time));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
// void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
//                                                      TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
//   const auto timeHorizon = finalTime - initTime;
//   modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

//   std::cout << "init time:" << initTime<< "\t" << " final time:" << finalTime << std::endl;
//   std::cout << modeSchedule << std::endl;

//   const scalar_t terrainHeight = initState(8) - 0.42; //For JYPro

//   footPlacementPlannerPtr_->update(modeSchedule, targetTrajectories, initTime, initState);

//   // Normal swing feet trajectory
//   // swingTrajectoryPtr_->update(modeSchedule, terrainHeight);

//   // For terrain aware swing feet trajectory planning
//   // swingTrajectoryPtr_->update(modeSchedule,
//   //                               footPlacementPlannerPtr_->getliftOffHeightSequence(),
//   //                               footPlacementPlannerPtr_->gettouchDownHeightSequence(),
//   //                               footPlacementPlannerPtr_->getfeetPlacementEvents(), initTime);

// }

template <typename T>
T square(T a) {
    return a * a;
}

template <typename SCALAR_T>
Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T>& q) {
    Eigen::Matrix<SCALAR_T, 3, 1> zyx;

    SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
    zyx(0) = std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
    zyx(1) = std::asin(as);
    zyx(2) = std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
    return zyx;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  // std::cout << "init time:" << initTime<< "\t" << " final time:" << finalTime << std::endl;
  //get current measure contact mode
  const size_t currentMode = stanceLeg2ModeNumber(terrainEstDataPtr_->stanceLegs);
  //update gait table and get predictive mode
  const auto timeHorizon = finalTime - initTime;
  // modeSchedule = tempModeSchedule_;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - 0.1, finalTime + 0.1);
  tempModeSchedule_ = modeSchedule;

  int modeIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime); // before or closet?
  const auto& mode = modeSchedule.modeSequence[modeIndex];

  // Terrain adaptation
  auto& targetState = targetTrajectories.stateTrajectory.back();
  const vector3_t& terrainParams = terrainEstDataPtr_->terrainParams.cast<scalar_t>();
  vector3_t terrainNormal = terrainParams;
  terrainNormal(2) = 1;

  const auto& stanceLegs = terrainEstDataPtr_->stanceLegs;
  vector3_t terrainZyx = quatToZyx(terrainEstDataPtr_->terrainQuat.cast<scalar_t>());
  if(abs(terrainZyx[1]) < 0.05) terrainZyx[1] = 0;
  if(abs(terrainZyx[2]) < 0.05) terrainZyx[2] = 0;

  auto swingConfig = swingTrajectoryPtr_->getConfig();
  const scalar_t distance2Terrain = swingConfig.comHeight; //For A1
  const scalar_t D2 = terrainParams[2] - distance2Terrain * terrainNormal.norm(); // D1 - h*sqrt(A^2 + B^2 + 1)
  const scalar_t zReference = - (terrainParams(0) * initState(6) + terrainParams(1) * initState(7) + D2);

  
  // if(targetTrajectories.timeTrajectory.size() >= 2){
  //   for(auto& stateTrajectory : targetTrajectories.stateTrajectory){
  //     stateTrajectory(8) = zReference;
  //     stateTrajectory(10) = 0.7*terrainZyx[1]; //pitch
  //     stateTrajectory(11) = 0.1*terrainZyx[2]; //roll
  //   }
  //   // std::cout << "desired pitch: " << targetTrajectories.stateTrajectory[1](10) << "\t desired roll: " << targetTrajectories.stateTrajectory[1](11) << std::endl;
  // }
  
  //Default Heuristic Footholds
  const auto& bodyPose = initState.segment<6>(6);
  const vector3_t& bodyPosition = bodyPose.head(3);
  const vector3_t& bodyEulerAngles = bodyPose.tail(3);
  Eigen::Matrix<scalar_t, 4, 4> _O_B_tfMatrix = Eigen::Matrix<scalar_t, 4, 4>::Identity();
  _O_B_tfMatrix.topLeftCorner(3,3) = ocs2::getRotationMatrixFromZyxEulerAngles(bodyEulerAngles);
  _O_B_tfMatrix.topRightCorner(3,1) = bodyPosition;

  feet_array_t<vector3_t> hipNominalPoints;
  const vector3_t height = vector3_t(0.0, 0.0, -distance2Terrain);

  scalar_t defaultFootXposition = swingConfig.defaultFootXposition, defaultFootYposition = swingConfig.defaultFootYposition;
  //{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  hipNominalPoints[0] = (_O_B_tfMatrix * (vector3_t(defaultFootXposition, defaultFootYposition, 0.0)  .homogeneous())).head(3) + height ;
  hipNominalPoints[1] = (_O_B_tfMatrix * (vector3_t(defaultFootXposition, -defaultFootYposition, 0.0) .homogeneous())).head(3) + height ;
  hipNominalPoints[2] = (_O_B_tfMatrix * (vector3_t(-defaultFootXposition, defaultFootYposition, 0.0) .homogeneous())).head(3) + height ;
  hipNominalPoints[3] = (_O_B_tfMatrix * (vector3_t(-defaultFootXposition, -defaultFootYposition, 0.0).homogeneous())).head(3) + height ;
  const vector_t q = initState.segment<18>(6);
  updateCentroidalDynamics(pinocchioInterface_, centroidalModelInfo_, q);
  // mappingPtr_->setPinocchioInterface(pinocchioInterface_);
  auto& data = pinocchioInterface_.getData();
  

  const auto currentqVelocity = mappingPtr_->getPinocchioJointVelocity(initState, vector_t::Zero(24));
  const auto currentVelocity = currentqVelocity.head(3);
  // std::cout << "currentVelocity: " << currentVelocity.transpose() << std::endl;
  // std::cout << "momentum: " << initState.segment<6>(0).transpose() << std::endl;

  const vector3_t positionAfter =  targetTrajectories.getDesiredState(initTime+0.1).segment<3>(6);
  const vector3_t positionNow =  targetTrajectories.getDesiredState(initTime).segment<3>(6);
  vector3_t commandedVelocity =  (positionAfter - positionNow) * 10;
  commandedVelocity.z() = 0;
  // std::cout << "commandedVelocity: " << commandedVelocity.transpose() << std::endl;

  // Normal swing feet trajectory
  feet_array_t<vector3_t> feetCurrentEEPositions;
  
  feet_array_t<std::vector<vector3_t>> feetTargeEEPositions;
  const contact_flag_t& currentContactFlags = modeNumber2StanceLeg(mode); // {LF, RF, LH, RH}

  for(int leg = 0; leg < 4; leg++){ //{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    feetCurrentEEPositions[leg] = footPlacementPlannerPtr_->getCurrentEEPosition(leg, initState);
    if(currentContactFlags[leg] == true)
      feetEETouchDownPositions_[leg] = feetCurrentEEPositions[leg];
    // std::cout << "leg: " << leg << "\t" << feetEETouchDownPositions_[leg].transpose() << "\t";
  }
  // std::cout << "\n";
  if(swingConfig.useDefaultHeuristicFootholds) {
    for(int leg = 0; leg < 4; leg++ ) {
      vector3_t footHold = hipNominalPoints[leg] + 0.1 * (currentVelocity - commandedVelocity) + 0.2*commandedVelocity;
      (*mpcNominalFeetholdsPtr_)[leg].clear();
      // (*mpcNominalFeetholdsPtr_)[leg].push_back(hipNominalPoints[leg]);
      (*mpcNominalFeetholdsPtr_)[leg].push_back(footHold);
      feetTargeEEPositions[leg].clear();
      feetTargeEEPositions[leg].push_back(footHold);
      // std::cout << "leg: " << leg << " footHold: " << footHold.transpose() << " normal: " << hipNominalPoints[leg].transpose() << "\n";
      
      footHold = hipNominalPoints[leg] + 0.1 * (currentVelocity - commandedVelocity) + 0.4*commandedVelocity;
      feetTargeEEPositions[leg].push_back(footHold);
      (*mpcNominalFeetholdsPtr_)[leg].push_back(footHold);
      // std::cout << "leg: " << leg << " footHold: " << footHold.transpose() << std::endl;
    }
    // swingTrajectoryPtr_->update(modeSchedule, feetCurrentEEPositions, initTime, feetTargeEEPositions); //这种情况下target需要有两个点

  }
  // else{
  //   swingTrajectoryPtr_->update(modeSchedule, footPlacementPlannerPtr_->getfeetPlacement(), initTime, feetCurrentEEPositions); // 默认的情况下不用这个函数？
  // }

  // Foothold planner
  footPlacementPlannerPtr_->setTargetPolygonVerteices(*mpcPolygonArrayPtr_, *mpcNominalFeetholdsPtr_);
  footPlacementPlannerPtr_->setTargetSwingHeight(*mpcSwingHeightPtr_);
  footPlacementPlannerPtr_->setTargetSwingMiddleTime(*mpcSwingMiddleTimePtr_);
  footPlacementPlannerPtr_->update(tempModeSchedule_, targetTrajectories, initTime, initState);
  if(swingConfig.useFootholdsAdjuestTrajectory) {
    footPlacementPlannerPtr_->setMpcTrajectoryAccordingToFootPlacement(initTime, modeSchedule, targetTrajectories, swingConfig.comHeight);
  }
  else {
    if(targetTrajectories.timeTrajectory.size() >= 2) {
    for(auto& stateTrajectory : targetTrajectories.stateTrajectory) {
      stateTrajectory(8) = zReference;
      stateTrajectory(10) = 0.7*terrainZyx[1]; //pitch
      stateTrajectory(11) = 0.1*terrainZyx[2]; //roll
    }
    // std::cout << "desired pitch: " << targetTrajectories.stateTrajectory[1](10) << "\t desired roll: " << targetTrajectories.stateTrajectory[1](11) << std::endl;
    }
  }
  // Swing Trajectory Planner
  swingTrajectoryPtr_->updateUsingMultiHeightAndSwingMiddleTime(tempModeSchedule_, footPlacementPlannerPtr_->getfeetPlacement(), initTime, feetEETouchDownPositions_,
      footPlacementPlannerPtr_->getSwingHeightSequence(), footPlacementPlannerPtr_->getSwingMiddleTimeSequence()); 

  // std::cout << "modifyReferences Done!" << "\n";
}

}  // namespace legged_robot
}  // namespace ocs2
