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
                                                             std::shared_ptr<feet_array_t<std::vector<vector3_t>>> mpcNominalFeetholdsPtr)
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
      mpcNominalFeetholdsPtr_(std::move(mpcNominalFeetholdsPtr)) { mappingPtr_->setPinocchioInterface(pinocchioInterface_); }

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
  // std::cout << "init time:" << initTime<< "\t" << " final time:" << finalTime << std::endl;
  modeSchedule = gaitSchedulePtr_->getModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);
  auto& targetState = targetTrajectories.stateTrajectory.back();
  const vector3_t& terrainParams = terrainEstDataPtr_->terrainParams.cast<scalar_t>();
  vector3_t terrainNormal = terrainParams;
  terrainNormal(2) = 1;

  std::cout << *terrainEstDataPtr_;

  vector3_t terrainRPY = terrainQuaternionToRPY_.quaternionToTotalRad(terrainEstDataPtr_->terrainQuat.cast<scalar_t>());
  // std::cout << "terrainRPY: " << terrainRPY.transpose() << std::endl;
  // std::cout << "terrainParam: " << terrainParams.transpose() << std::endl;

  const scalar_t distance2Terrain = 0.41; //For X20
  const scalar_t D2 = terrainParams[2] - distance2Terrain * terrainNormal.norm(); // D1 - h*sqrt(A^2 + B^2 + 1)
  const scalar_t zReference = - (terrainParams(0) * initState(6) + terrainParams(1) * initState(7) + D2);
  // std::cout << "zReference: " << zReference << "\t D2: " << D2 << " terrainNormal.norm: " << terrainNormal.norm() << std::endl;
  // std::cout << "intiState: " << initState.segment(6, 18).transpose() << std::endl;

  // std::cout << targetTrajectories;

  if(targetTrajectories.timeTrajectory.size() >= 2){
    // targetState(8) = zReference;
    // targetState(10) = terrainRPY[1]; //pitch
    // targetState(11) = terrainRPY[0]; //roll
    for(auto& stateTrajectory : targetTrajectories.stateTrajectory){
      stateTrajectory(8) = zReference;
      stateTrajectory(10) = terrainRPY[1]; //pitch
      stateTrajectory(11) = terrainRPY[0]; //roll
    }
    // targetTrajectories.stateTrajectory[1][10] = terrainRPY[1]; // pitch
    // targetTrajectories.stateTrajectory[1][11] = terrainRPY[0]; // roll
    // targetTrajectories.stateTrajectory[1][8] = zReference; // z

    // std::cout << "######## modify target state ########\n"; 
  }



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
  const vector3_t height = vector3_t(0.0, 0.0, -0.476);
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
  std::cout << "currentVelocity: " << currentVelocity.transpose() << std::endl;
  // std::cout << "momentum: " << initState.segment<6>(0).transpose() << std::endl;


  const vector3_t positionAfter =  targetTrajectories.getDesiredState(initTime+0.1).segment<3>(6);
  const vector3_t positionNow =  targetTrajectories.getDesiredState(initTime).segment<3>(6);
  const vector3_t commandedVelocity =  (positionAfter - positionNow) * 10;
  // std::cout << "commandedVelocity: " << commandedVelocity.transpose() << std::endl;



  std::cout << "commandedVelocity: " << commandedVelocity.transpose() << std::endl;




  // abort();

  // Normal swing feet trajectory
  // swingTrajectoryPtr_->update(modeSchedule, -0.44);
  // swingTrajectoryPtr_->update(modeSchedule, terrainEstDataPtr_->feetHeight.cast<scalar_t>());
  feet_array_t<vector3_t> feetCurrentEEPositions;
  feet_array_t<std::vector<vector3_t>> feetTargeEEPositions;
  for(int leg = 0; leg < 4; leg++){
    feetCurrentEEPositions[leg] = footPlacementPlannerPtr_->getCurrentEEPosition(leg, initState);
  }
  if(0){
    for(int leg = 0; leg < 4; leg++ ){
      vector3_t footHold = hipNominalPoints[leg] + 0.21 * (currentVelocity - commandedVelocity) + 0.15*currentVelocity;
      (*mpcNominalFeetholdsPtr_)[leg].clear();
      (*mpcNominalFeetholdsPtr_)[leg].push_back(hipNominalPoints[leg]);
      (*mpcNominalFeetholdsPtr_)[leg].push_back(footHold);
      feetTargeEEPositions[leg].clear();
      feetTargeEEPositions[leg].push_back(footHold);
      std::cout << "leg: " << leg << " footHold: " << footHold.transpose() << std::endl;

      footHold = hipNominalPoints[leg] + 0.21 * (currentVelocity - commandedVelocity) + 0.3*currentVelocity;
      feetTargeEEPositions[leg].push_back(footHold);
      (*mpcNominalFeetholdsPtr_)[leg].push_back(footHold);


      std::cout << "leg: " << leg << " footHold: " << footHold.transpose() << std::endl;
    }
    // swingTrajectoryPtr_->update(modeSchedule, feetCurrentEEPositions, initTime, feetTargeEEPositions); //这种情况下target需要有两个点

  }
  // else{
  //   swingTrajectoryPtr_->update(modeSchedule, footPlacementPlannerPtr_->getfeetPlacement(), initTime, feetCurrentEEPositions); // 默认的情况下不用这个函数？
  // }
  footPlacementPlannerPtr_->setTargetPolygonVerteices(*mpcPolygonArrayPtr_, *mpcNominalFeetholdsPtr_);
  footPlacementPlannerPtr_->update(modeSchedule, targetTrajectories, initTime, initState);
  swingTrajectoryPtr_->update(modeSchedule, footPlacementPlannerPtr_->getfeetPlacement(), initTime, feetCurrentEEPositions); // 默认的情况下不用这个函数？
  // swingTrajectoryPtr_->update(modeSchedule, feetCurrentEEPositions, initTime, feetTargeEEPositions); 


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
