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
    : LeggedRobotReferenceManager(TargetTrajectories(), ModeSchedule(), TargetFeetPlacement()),
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
void SwitchedModelReferenceManager::modifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  std::cout << "init time:" << initTime<< "\t" << " final time:" << finalTime << std::endl;
  std::cout << modeSchedule << std::endl;

  const scalar_t terrainHeight = initState(8) - 0.42; //For JYPro

  footPlacementPlannerPtr_->update(modeSchedule, targetTrajectories, initTime, initState);

  // Normal swing feet trajectory
  // swingTrajectoryPtr_->update(modeSchedule, terrainHeight);

  // For terrain aware swing feet trajectory planning
  // swingTrajectoryPtr_->update(modeSchedule,
  //                               footPlacementPlannerPtr_->getliftOffHeightSequence(),
  //                               footPlacementPlannerPtr_->gettouchDownHeightSequence(),
  //                               footPlacementPlannerPtr_->getfeetPlacementEvents(), initTime);

}
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
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule,
                                                     TargetFeetPlacement& targetFeetPlacement) {
  // std::cout << "init time:" << initTime<< "\t" << " final time:" << finalTime << std::endl;
  //get current measure contact mode
  const size_t currentMode = stanceLeg2ModeNumber(terrainEstDataPtr_->stanceLegs);
  //update gait table and get predictive mode
  const auto timeHorizon = finalTime - initTime;
  // modeSchedule = tempModeSchedule_;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  tempModeSchedule_ = modeSchedule;

  // if(isLateTouchdown_){
  //   if(initTime - lateTouchdownTime_ > 0.1) {
  //     isLateTouchdown_ = false;
  //   }
  // }
  // // isLateTouchdown_ = false;

  // std::cout << "### Current modeSchedule:" << std::endl;
  // std::cout << modeSchedule;
  int modeIndex = lookup::findIndexInTimeArray(modeSchedule.eventTimes, initTime); // before or closet?
  const auto& mode = modeSchedule.modeSequence[modeIndex];
  // std::cout << "modeIndex: " << modeIndex << "\t";
  // std::cout << "actual mode: " << currentMode << " predictive mode:" << mode  << std::endl;
  // if(modeIndex > 0) {
  //   if(modeSchedule.eventTimes[modeIndex] - initTime > initTime - modeSchedule.eventTimes[modeIndex-1]) {
  //     modeIndex--;
  //   }
  // }


  // if (mode != currentMode && isLateTouchdown_ == false) {
  //   const contact_flag_t& predictiveContactFlags = modeNumber2StanceLeg(mode);
  //   const contact_flag_t& actualContactFlags = terrainEstDataPtr_->stanceLegs;
  //   insertContactFlags_ = {true, true, true, true}; // default insert mode is all legs in contact, prevent the mode is zero.
  //   for(int leg = 0; leg < 4; leg++) {
  //     if (predictiveContactFlags[leg] == true && actualContactFlags[leg] == false) { //late touchdown
  //       std::cout << "################# late touchdown: leg " << leg << " #################" << std::endl;
  //       insertContactFlags_[leg] = false;
  //       isLateTouchdown_ = true;
  //       lateTouchdownTime_ = initTime;
  //       insertContactTimes_ = 2;
  //     }
  //   }
  // }

  // if(mode == currentMode){
  //   isLateTouchdown_ = false;
  // }
  //   // insert the new mode
  // if(isLateTouchdown_ && (insertContactTimes_ > 0) && (mode != currentMode)) {
  //     // delay the mode after the late touchdown
  //     for(int i = modeIndex; i < modeSchedule.eventTimes.size(); i++) {
  //       modeSchedule.eventTimes[i] += 0.05;
  //     }
  //     // for(int i = 0; i < modeIndex; i++) {
  //     //   modeSchedule.eventTimes[i] -= 0.02;
  //     // }

  //     const auto& modeAfterRecover = modeSchedule.modeSequence[modeIndex+1]; // maybe change to closet mode according to time.

  //     modeSchedule.eventTimes.insert(modeSchedule.eventTimes.begin() + modeIndex, lateTouchdownTime_ - 0.01); // recover mode start time
  //     modeSchedule.eventTimes.insert(modeSchedule.eventTimes.begin() + modeIndex + 1, lateTouchdownTime_ + 0.05); // recover mode end time
  //     modeSchedule.modeSequence.insert(modeSchedule.modeSequence.begin() + modeIndex + 1, stanceLeg2ModeNumber(insertContactFlags_));
  //     modeSchedule.modeSequence.insert(modeSchedule.modeSequence.begin() + modeIndex + 2, modeAfterRecover);
  //     std::cout << "### After modeSchedule:" << std::endl;
  //     std::cout << modeSchedule;
  //     insertContactTimes_--;
  //     // gaitSchedulePtr_->setModeSchedule(modeSchedule);
  //   }
  




  auto& targetState = targetTrajectories.stateTrajectory.back();
  const vector3_t& terrainParams = terrainEstDataPtr_->terrainParams.cast<scalar_t>();
  vector3_t terrainNormal = terrainParams;
  terrainNormal(2) = 1;

  const auto& stanceLegs = terrainEstDataPtr_->stanceLegs;

  vector3_t terrainZyx = quatToZyx(terrainEstDataPtr_->terrainQuat.cast<scalar_t>());
  // std::cout << "terrainZyx: " << terrainZyx.transpose();
  
  // std::cout << "terrainParam: " << terrainParams.transpose() << std::endl;
  if(abs(terrainZyx[1]) < 0.05) terrainZyx[1] = 0;
  if(abs(terrainZyx[2]) < 0.05) terrainZyx[2] = 0;
  // std::cout << "\tterrainZyx After: " << terrainZyx.transpose() << std::endl;
  // std::cout << "stanceLegs: " << stanceLegs[0] << stanceLegs[1] << stanceLegs[2] << stanceLegs[3] << "\n";
  const scalar_t distance2Terrain = 0.436; //For X20
  const scalar_t D2 = terrainParams[2] - distance2Terrain * terrainNormal.norm(); // D1 - h*sqrt(A^2 + B^2 + 1)
  const scalar_t zReference = - (terrainParams(0) * initState(6) + terrainParams(1) * initState(7) + D2);
  // std::cout << "zReference: " << zReference << "\t D2: " << D2 << " terrainNormal.norm: " << terrainNormal.norm() << std::endl;
  // std::cout << "intiState: " << initState.segment(6, 18).transpose() << std::endl;

  // std::cout << targetTrajectories;

  if(targetTrajectories.timeTrajectory.size() >= 2){
    // targetState(8) = zReference;
    // targetState(10) = terrainZyx[1]; //pitch
    // targetState(11) = terrainZyx[0]; //roll
    for(auto& stateTrajectory : targetTrajectories.stateTrajectory){
      stateTrajectory(8) = zReference;
      stateTrajectory(10) = 0.7*terrainZyx[1]; //pitch
      stateTrajectory(11) = 0.1*terrainZyx[2]; //roll
    }
    // targetTrajectories.stateTrajectory[1][10] = terrainZyx[1]; // pitch
    // targetTrajectories.stateTrajectory[1][11] = 0.5*terrainZyx[0]; // roll
    // targetTrajectories.stateTrajectory[1][8] = zReference; // z

    // std::cout << "######## modify target state ########\n"; 
  }
    // std::cout << targetTrajectories;



  //   std::cout << "######## modify target state ########\n";
  // }

  // std::cout << "targetTrajectories:" << targetTrajectories.stateTrajectory.back().segment(6,6).transpose() << std::endl;

  // std::cout << "init time:" << initTime<< "\t" << " final time:" << finalTime << std::endl;
  // std::cout << modeSchedule << std::endl;

  // const scalar_t terrainHeight = initState(8) - 0.42; //For JYPro

  // std::cout << "targetFeetPlacement L size:" << targetFeetPlacement.targetFeetPlacemetLeft_.size() << "\n";
  // std::cout << "targetFeetPlacement R size:" << targetFeetPlacement.targetFeetPlacemetRight_.size() << "\n";
  // const auto& leftFront = targetFeetPlacement.targetFeetPlacemetLeftFront_;
  // const auto& rightFront = targetFeetPlacement.targetFeetPlacemetRightFront_;
  // const auto& leftBack = targetFeetPlacement.targetFeetPlacemetLeftBack_;
  // const auto& rightBack = targetFeetPlacement.targetFeetPlacemetRightBack_;
  // for(const auto& left_i : leftFront) {
  //   std::cout << "left_i:" << left_i.transpose() << "\n";
  // }
  // for(const auto& right_i : right) {
  //   std::cout << "right_i:" << right_i.transpose() << "\n";
  // }
  // footPlacementPlannerPtr_->setTargetPoints(leftFront, rightFront, leftBack, rightBack);
  // std::cout << "mpcPolygonArrayPtr_[0][0][0]: " << (*mpcPolygonArrayPtr_)[0][0][0].transpose() << std::endl;
  LeggedIKSolverPtr_->setBodyState(initState.segment<6>(6));
  const auto& _O_B_tfMatrix =  LeggedIKSolverPtr_->getBodyTfMatrix();
  feet_array_t<vector3_t> hipNominalPoints;
  const vector3_t height = vector3_t(0.0, 0.0, -distance2Terrain);
  //{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  hipNominalPoints[0] = (_O_B_tfMatrix * (vector3_t(__FOOT_X__, __FOOT_Y__, 0.0)  .homogeneous())).head(3) + height ;
  hipNominalPoints[1] = (_O_B_tfMatrix * (vector3_t(__FOOT_X__, -__FOOT_Y__, 0.0) .homogeneous())).head(3) + height ;
  hipNominalPoints[2] = (_O_B_tfMatrix * (vector3_t(-__FOOT_X__, __FOOT_Y__, 0.0) .homogeneous())).head(3) + height ;
  hipNominalPoints[3] = (_O_B_tfMatrix * (vector3_t(-__FOOT_X__, -__FOOT_Y__, 0.0).homogeneous())).head(3) + height ;
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
  const vector3_t commandedVelocity =  (positionAfter - positionNow) * 10;
  // std::cout << "commandedVelocity: " << commandedVelocity.transpose() << std::endl;

  // std::cout << "commandedVelocity: " << commandedVelocity.transpose() << std::endl;
  // abort();

  // Normal swing feet trajectory
  // swingTrajectoryPtr_->update(modeSchedule, -0.44);
  // swingTrajectoryPtr_->update(modeSchedule, terrainEstDataPtr_->feetHeight.cast<scalar_t>());
  feet_array_t<vector3_t> feetCurrentEEPositions;
  static feet_array_t<vector3_t> feetEETouchDownPositions;
  feet_array_t<std::vector<vector3_t>> feetTargeEEPositions;
  const contact_flag_t& currentContactFlags = modeNumber2StanceLeg(mode); // {LF, RF, LH, RH}

  for(int leg = 0; leg < 4; leg++){ //{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
    feetCurrentEEPositions[leg] = footPlacementPlannerPtr_->getCurrentEEPosition(leg, initState);
    if(currentContactFlags[leg] == true)
      feetEETouchDownPositions[leg] = feetCurrentEEPositions[leg];
  }
  if(0){
    for(int leg = 0; leg < 4; leg++ ){
      vector3_t footHold = hipNominalPoints[leg] + 0.21 * (currentVelocity - commandedVelocity) + 0.2*commandedVelocity;
      (*mpcNominalFeetholdsPtr_)[leg].clear();
      // (*mpcNominalFeetholdsPtr_)[leg].push_back(hipNominalPoints[leg]);
      (*mpcNominalFeetholdsPtr_)[leg].push_back(footHold);
      feetTargeEEPositions[leg].clear();
      feetTargeEEPositions[leg].push_back(footHold);
      // std::cout << "leg: " << leg << " footHold: " << footHold.transpose() << std::endl;

      footHold = hipNominalPoints[leg] + 0.21 * (currentVelocity - commandedVelocity) + 0.4*commandedVelocity;
      feetTargeEEPositions[leg].push_back(footHold);
      (*mpcNominalFeetholdsPtr_)[leg].push_back(footHold);


      // std::cout << "leg: " << leg << " footHold: " << footHold.transpose() << std::endl;
    }
    // swingTrajectoryPtr_->update(modeSchedule, feetCurrentEEPositions, initTime, feetTargeEEPositions); //这种情况下target需要有两个点

  }
  // else{
  //   swingTrajectoryPtr_->update(modeSchedule, footPlacementPlannerPtr_->getfeetPlacement(), initTime, feetCurrentEEPositions); // 默认的情况下不用这个函数？
  // }
  footPlacementPlannerPtr_->setTargetPolygonVerteices(*mpcPolygonArrayPtr_, *mpcNominalFeetholdsPtr_);
  footPlacementPlannerPtr_->setTargetSwingHeight(*mpcSwingHeightPtr_);
  footPlacementPlannerPtr_->setTargetSwingMiddleTime(*mpcSwingMiddleTimePtr_);
  footPlacementPlannerPtr_->update(modeSchedule, targetTrajectories, initTime, initState);
  // swingTrajectoryPtr_->update(modeSchedule, footPlacementPlannerPtr_->getfeetPlacement(), initTime, feetEETouchDownPositions,
  //     footPlacementPlannerPtr_->getSwingHeightSequence(), footPlacementPlannerPtr_->getSwingMiddleTimeSequence(), isLateTouchdown_); // 默认的情况下不用这个函数？
  swingTrajectoryPtr_->updateUsingMultiHeightAndSwingMiddleTime(modeSchedule, footPlacementPlannerPtr_->getfeetPlacement(), initTime, feetEETouchDownPositions,
      footPlacementPlannerPtr_->getSwingHeightSequence(), footPlacementPlannerPtr_->getSwingMiddleTimeSequence()); 
  // swingTrajectoryPtr_->update(modeSchedule, feetCurrentEEPositions, initTime, feetTargeEEPositions); 
  // swingTrajectoryPtr_->update(modeSchedule, footPlacementPlannerPtr_->getfeetPlacement(), initTime, feetEETouchDownPositions, isLateTouchdown_);

  // swingTrajectoryPtr_->update(modeSchedule, 0.03);


  // std::cout << *terrainEstDataPtr_ << std::endl;
  // For terrain aware swing feet trajectory planning
  // this part makes the robot performance worse. something wrong in this part.
  // swingTrajectoryPtr_->update(modeSchedule,
  //                               footPlacementPlannerPtr_->getliftOffHeightSequence(),
  //                               footPlacementPlannerPtr_->gettouchDownHeightSequence(),
  //                               footPlacementPlannerPtr_->getfeetPlacementEvents(), initTime);

  // std::cout << "modifyReferences Done!" << "\n";

}

}  // namespace legged_robot
}  // namespace ocs2
