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

#include "ocs2_jypro/cost/LeggedRobotStateInputQuadraticCost.h"
#include "ocs2_jypro/LeggedRobotPreComputation.h"

#include <ocs2_jypro/common/utils.h>

#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include <ocs2_robotic_tools/common/RotationTransforms.h>

namespace ocs2 {
namespace legged_robot {
inline matrix3_t rpyTORotateMat(vector3_t rpy);
static Eigen::Vector2d integralError_ = Eigen::Vector2d::Zero();
vector3_t xNominalOrientation_ = vector3_t::Zero();
/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotStateInputQuadraticCost::LeggedRobotStateInputQuadraticCost(matrix_t Q, matrix_t R, CentroidalModelInfo info,
                                                                       const SwitchedModelReferenceManager& referenceManager,
                                                                       std::shared_ptr<LeggedIKSolver> leggedIKSolverPtr,
                                                                       bool useIKresult)
    : LeggedRobotQuadraticStateInputCost(std::move(Q), std::move(R)), info_(std::move(info)), referenceManagerPtr_(&referenceManager), 
    leggedIKSolverPtr_(std::move(leggedIKSolverPtr)), useIKresult_(useIKresult) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotStateInputQuadraticCost* LeggedRobotStateInputQuadraticCost::clone() const {
  return new LeggedRobotStateInputQuadraticCost(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<vector_t, vector_t> LeggedRobotStateInputQuadraticCost::getStateInputDeviation(
    scalar_t time, const vector_t& state, const vector_t& input, const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
  // const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  // vector_t xNominal = targetTrajectories.getDesiredState(time);
  // const vector_t uNominal = weightCompensatingInput(info_, contactFlags);
  // vector3_t xNominalOrientation = xNominal.segment<3>(9);
  // vector3_t xOrientation = state.segment<3>(9);

  // makeEulerAnglesUnique(xNominalOrientation);
  // const auto yaw = moduloAngleWithReference(xNominalOrientation[0], xNominalOrientation_[0]);
  // xNominalOrientation[0] = yaw;
  // // makeEulerAnglesUnique(xOrientation);
  // // std::cout << "xNominalOrientation: " << xNominalOrientation.transpose() << std::endl;
  // // std::cout << "xOrientation: " << xOrientation.transpose() << std::endl;

  // const matrix3_t R = getRotationMatrixFromZyxEulerAngles(xOrientation);
  // const matrix3_t RNominal = getRotationMatrixFromZyxEulerAngles(xNominalOrientation);
  // const vector3_t errVec = rotationErrorInWorld(R, RNominal).reverse(); // cant more than 90 degree.
  // // const matrix3_t err = (R * RNominal.transpose()).log();
  // // const vector3_t errVec = vector3_t(err(2, 1), err(0, 2), err(1, 0));

  // xNominal.segment<3>(9) = xNominalOrientation;
  // vector_t xDeviation = state - xNominal;
  // // xDeviation.segment<3>(9) = errVec;

  // const auto currentPoseError = xDeviation.segment<6>(6);
  // std::cout << "currentPoseError: " << currentPoseError.transpose() << std::endl;
  // xDeviation.segment<6>(6) = 2*currentPoseError;
  // // std::cout << "xDeviation: " << xDeviation.transpose() << std::endl;
  // xNominalOrientation_ = xNominalOrientation;
  // return {xDeviation, input - uNominal};

  const auto contactFlags = referenceManagerPtr_->getContactFlags(time);
  vector_t xNominal = targetTrajectories.getDesiredState(time);
  const vector_t uNominal = weightCompensatingInput(info_, contactFlags);
  const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
  feet_array_t<vector3_t> referenceQj; //{"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};
  const auto& getEEReference = preCompLegged.getEEReference();

    // std::cout << "time: " << time << " base xyz: " <<  xNominal.segment<3>(6).transpose() << std::endl; // use target trajectory.
  if(useIKresult_){
    leggedIKSolverPtr_->setBodyState(xNominal.segment<6>(6));
    for (size_t i = 0; i < 4; i++) {
      if(contactFlags[i] == 1)
        referenceQj[i] = xNominal.segment<3>(12+3*i);
      else{
        referenceQj[i] = leggedIKSolverPtr_->solveIK(getEEReference[i], i);
        if(referenceQj[i].hasNaN()){
          referenceQj[i] = xNominal.segment<3>(12+3*i);
          std::cerr << "######### IK solver Failed #########\n";
        }
      }
    }
  
    Eigen::Matrix<scalar_t, 12, 1> qj; //[LF, LH, RF, RH] 

    qj << referenceQj[0], referenceQj[2], referenceQj[1], referenceQj[3];
    // xNominal.tail(12) = qj;
    // std::cout << "IK result: " << qj.transpose() << "\n";
  }
  // std::cout << "qj: " << qj.transpose() << "\n";

  vector_t xDeviation = state - xNominal;
  // std::cout << "xNominal: " << xNominal.transpose() << std::endl;
  // std::cout << "xDeviation: " << xDeviation.transpose() << std::endl;

  const auto currentPoseError = xDeviation.segment<2>(6);
  // // static Eigen::Vector2d integralError;
  // integralError_ = integralError_+currentPoseError*0.01;
  // if(integralError_[0] > 1)
  //   integralError_[0] = 1;
  // if(integralError_[0] < -1)
  //   integralError_[0] = -1;
  // if(integralError_[1] > 1)
  //   integralError_[1] = 1;
  // if(integralError_[1] < -1)
  //   integralError_[1] = -1;
  // std::cout << "integralError_: " << integralError_.transpose() << std::endl;

  xDeviation.segment<2>(6) = currentPoseError + 10*integralError_;
  return {xDeviation, input - uNominal};

}


inline matrix3_t rpyTORotateMat(vector3_t rpy){
    const scalar_t roll = rpy(0);
    const scalar_t pitch = rpy(1);
    const scalar_t yaw = rpy(2);
    using namespace std;
    matrix3_t RotateMatrix, R_roll, R_pitch, R_yaw;
    R_roll <<  1., 0., 0., 
               0., cos(roll), -sin(roll),
               0., sin(roll), cos(roll);
    R_pitch << cos(pitch), 0, sin(pitch),
               0., 1., 0.,
               -sin(pitch), 0., cos(pitch);
    R_yaw << cos(yaw), -sin(yaw), 0.,
             sin(yaw), cos(yaw), 0.,
             0., 0., 1.;
    RotateMatrix = R_yaw * R_pitch * R_roll;
    return RotateMatrix;
}

}  // namespace legged_robot
}  // namespace ocs2 
