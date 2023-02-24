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

#include <iostream>
#include <string>

#include <pinocchio/fwd.hpp>  // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "ocs2_jypro/LeggedRobotInterface.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Display.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include "ocs2_jypro/LeggedRobotPreComputation.h"
#include "ocs2_jypro/constraint/FrictionConeConstraint.h"
#include "ocs2_jypro/constraint/NormalVelocityConstraintCppAd.h"
#include "ocs2_jypro/constraint/ZeroForceConstraint.h"
#include "ocs2_jypro/constraint/ZeroVelocityConstraintCppAd.h"
#include "ocs2_jypro/constraint/StateOnlyFootPlacementConstraintCppAD.h"
#include "ocs2_jypro/constraint/CBFFootPlacementConstraintCppAD.h"
#include "ocs2_jypro/cost/LeggedRobotStateInputQuadraticCost.h"
#include "ocs2_jypro/dynamics/LeggedRobotDynamicsAD.h"
#include "ocs2_jypro/cost/LeggedRobotEndEffectorCost.h"
#include "ocs2_jypro/foot_planner/LeggedIKSolver.h"


// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>


namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInterface::LeggedRobotInterface(const std::string& taskFile, const std::string& urdfFile, const std::string& referenceFile) {
  // check that task file exists
  boost::filesystem::path taskFilePath(taskFile);
  if (boost::filesystem::exists(taskFilePath)) {
    std::cerr << "[LeggedRobotInterface] Loading task file: " << taskFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] Task file not found: " + taskFilePath.string());
  }
  // check that urdf file exists
  boost::filesystem::path urdfFilePath(urdfFile);
  if (boost::filesystem::exists(urdfFilePath)) {
    std::cerr << "[LeggedRobotInterface] Loading Pinocchio model from: " << urdfFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] URDF file not found: " + urdfFilePath.string());
  }
  // check that targetCommand file exists
  boost::filesystem::path referenceFilePath(referenceFile);
  if (boost::filesystem::exists(referenceFilePath)) {
    std::cerr << "[LeggedRobotInterface] Loading target command settings from: " << referenceFilePath << std::endl;
  } else {
    throw std::invalid_argument("[LeggedRobotInterface] targetCommand file not found: " + referenceFilePath.string());
  }

  bool verbose;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.verbose", verbose);

  // load setting from loading file
  modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
  ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
  mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
  rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
  sqpSettings_ = multiple_shooting::loadSettings(taskFile, "multiple_shooting", verbose);

  // OptimalConrolProblem
  setupOptimalConrolProblem(taskFile, urdfFile, referenceFile, verbose);

  // initial state
  initialState_.setZero(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> LeggedRobotInterface::loadGaitSchedule(const std::string& taskFile) {
  const auto initModeSchedule = loadModeSchedule(taskFile, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(taskFile, "defaultModeSequenceTemplate", false);

  const auto defaultGait = [&] {
    Gait gait{};
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    return gait;
  }();

  // display
  std::cerr << "\nInitial Modes Schedule: \n" << initModeSchedule << std::endl;
  std::cerr << "\nDefault Modes Sequence Template: \n" << defaultModeSequenceTemplate << std::endl;

  return std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings_.phaseTransitionStanceTime);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInterface::setupOptimalConrolProblem(const std::string& taskFile, const std::string& urdfFile,
                                                     const std::string& referenceFile, bool verbose) {
  // PinocchioInterface
  pinocchioInterfacePtr_.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames))); //DQWANG:URDF MODEL -> PINOCCHIO MODEL

  // CentroidalModelInfo
  centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
      *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile),
      centroidal_model::loadDefaultJointState(pinocchioInterfacePtr_->getModel().nq - 6, referenceFile), modelSettings_.contactNames3DoF,
      modelSettings_.contactNames6DoF);
  // Swing trajectory planner
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(
      new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config"), 4));

  // Foot placement planner
  CentroidalModelPinocchioMapping pinocchioMapping(getCentroidalModelInfo());
  PinocchioEndEffectorKinematics endEffectorKinematics(*pinocchioInterfacePtr_, pinocchioMapping,
                                                       modelSettings().contactNames3DoF);
  std::unique_ptr<FootPlacementPlanner> footPlacementPlanner(
      new FootPlacementPlanner(*pinocchioInterfacePtr_, endEffectorKinematics, getCentroidalModelInfo(), 4));
  
  std::unique_ptr<legged::LeggedIKSolver> leggedIKSolverPtr_(new legged::LeggedIKSolver(*pinocchioInterfacePtr_, getCentroidalModelInfo(), endEffectorKinematics));
  
  std::shared_ptr<TerrainEstData> terrainEstDataPtr = std::make_shared<TerrainEstData>();
  // Mode schedule manager
  referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(taskFile), std::move(swingTrajectoryPlanner), 
                                                                            std::move(footPlacementPlanner), terrainEstDataPtr);

  // Optimal control problem
  problemPtr_.reset(new OptimalControlProblem);

  // Dynamics
  bool useAnalyticalGradientsDynamics = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsDynamics", useAnalyticalGradientsDynamics);
  std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
  if (useAnalyticalGradientsDynamics) {
    throw std::runtime_error("[LeggedRobotInterface::setupOptimalConrolProblem] The analytical dynamics class is not yet implemented.");
  } else {
    const std::string modelName = "dynamics";
    dynamicsPtr.reset(new LeggedRobotDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, modelName, modelSettings_));
  }

  problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

  // Cost terms
  problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_));

  // Constraint terms
  // friction cone settings
  scalar_t frictionCoefficient = 0.7;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig, barrierPenaltyConfig_;
  std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(taskFile);
  barrierPenaltyConfig_ = loadFootPlacementSettings(taskFile);

  bool useAnalyticalGradientsConstraints = false;
  loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsConstraints", useAnalyticalGradientsConstraints);
  for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) {
    const std::string& footName = modelSettings_.contactNames3DoF[i];

    std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;

    const auto infoCppAd = centroidalModelInfo_.toCppAd();
    const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
    auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
      const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
      updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
    };

    eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {footName},
                                                                  centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                  velocityUpdateCallback, footName, modelSettings_.modelFolderCppAd,
                                                                  modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
    

    problemPtr_->costPtr->add(footName + "_endEffectorTrackingCost", 
                              getEndEffectorTrackingCost(taskFile, *eeKinematicsPtr, footName + "_endEffectorTrackingCost" , i,
                              modelSettings_.modelFolderCppAd, modelSettings_.recompileLibrariesCppAd));
    // std::cout << "add done!\n";
    problemPtr_->softConstraintPtr->add(footName + "_frictionCone",
                                        getFrictionConeConstraint(i, frictionCoefficient, barrierPenaltyConfig));

    problemPtr_->stateSoftConstraintPtr->add(footName + "_placement",
                                             getStateOnlyFootPlacementConstraint(*eeKinematicsPtr, footName + "_placementConstraint",
                                              i, barrierPenaltyConfig_)
                                             );
    // problemPtr_->softConstraintPtr->add(footName + "_CBFplacement",
    //                                             getCBFFootPlacementConstraint(*eeKinematicsPtr, 
    //                                             footName + "_CBFplacementConstraint",i, barrierPenaltyConfig));

    problemPtr_->equalityConstraintPtr->add(footName + "_zeroForce", getZeroForceConstraint(i));
    problemPtr_->equalityConstraintPtr->add(footName + "_zeroVelocity",
                                            getZeroVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));
    problemPtr_->equalityConstraintPtr->add(footName + "_normalVelocity",
                                            getNormalVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));
  }

  // Pre-computation
  problemPtr_->preComputationPtr.reset(new LeggedRobotPreComputation(*pinocchioInterfacePtr_, centroidalModelInfo_,
                                                                     *referenceManagerPtr_->getSwingTrajectoryPlanner(), 
                                                                     *referenceManagerPtr_->getFootPlacementPlanner(),
                                                                     std::move(leggedIKSolverPtr_),
                                                                     terrainEstDataPtr,
                                                                     modelSettings_));

  // Rollout
  rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

  // Initialization
  constexpr bool extendNormalizedMomentum = true;
  initializerPtr_.reset(new LeggedRobotInitializer(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInterface::initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info, matrix_t& R) {
  vector_t initialState(centroidalModelInfo_.stateDim);
  loadData::loadEigenMatrix(taskFile, "initialState", initialState);

  const auto& model = pinocchioInterfacePtr_->getModel();
  auto& data = pinocchioInterfacePtr_->getData();
  const auto q = centroidal_model::getGeneralizedCoordinates(initialState, centroidalModelInfo_);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);

  matrix_t baseToFeetJacobians(3 * info.numThreeDofContacts, 12);
  for (size_t i = 0; i < info.numThreeDofContacts; i++) {
    matrix_t jacobianWorldToContactPointInWorldFrame = matrix_t::Zero(6, info.generalizedCoordinatesNum);
    pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]), pinocchio::LOCAL_WORLD_ALIGNED,
                                jacobianWorldToContactPointInWorldFrame);

    baseToFeetJacobians.block(3 * i, 0, 3, 12) = (jacobianWorldToContactPointInWorldFrame.topRows<3>()).block(0, 6, 3, 12);
  }

  const size_t totalContactDim = 3 * info.numThreeDofContacts;
  R.block(totalContactDim, totalContactDim, 12, 12) =
      (baseToFeetJacobians.transpose() * R.block(totalContactDim, totalContactDim, 12, 12) * baseToFeetJacobians).eval(); //dqwang: 这里维数感觉有问题
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info) {
  matrix_t Q(info.stateDim, info.stateDim);
  loadData::loadEigenMatrix(taskFile, "Q", Q);
  matrix_t R(info.inputDim, info.inputDim);
  loadData::loadEigenMatrix(taskFile, "R", R);

  initializeInputCostWeight(taskFile, info, R);

  if (display_) {
    std::cerr << "\n #### Base Tracking Cost Coefficients: ";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << Q << "\n";
    std::cerr << "R:\n" << R << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::unique_ptr<StateInputCost>(new LeggedRobotStateInputQuadraticCost(std::move(Q), std::move(R), info, *referenceManagerPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::getEndEffectorTrackingCost(
                                                      const std::string& taskFile, 
                                                      const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                      const std::string& modelName, size_t contactPointIndex,
                                                      const std::string& modelFolderCppAd, bool recompileCppAd) {
  matrix_t Q(3, 3);
  loadData::loadEigenMatrix(taskFile, "Qee", Q);
  matrix_t R(3, 3);
  loadData::loadEigenMatrix(taskFile, "Ree", R);


  if (display_) {
    std::cerr << "\n #### Base EndEffector Tracking Cost Coefficients For leg: " << contactPointIndex;
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "Q:\n" << Q << "\n";
    std::cerr << "R:\n" << R << "\n";
    std::cerr << " #### =============================================================================\n";
  }

  return std::unique_ptr<StateInputCost>(new LeggedRobotEndEffectorCost(std::move(Q), std::move(R), eeKinematics,
                                         contactPointIndex, centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim, 
                                         modelName, modelFolderCppAd, recompileCppAd));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, RelaxedBarrierPenalty::Config> LeggedRobotInterface::loadFrictionConeSettings(const std::string& taskFile) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "frictionConeSoftConstraint.";

  scalar_t frictionCoefficient = 1.0;
  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  if (display_) {
    std::cerr << "\n #### Friction Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", display_);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", display_);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", display_);
  if (display_) {
    std::cerr << " #### =============================================================================\n";
  }

  return {frictionCoefficient, std::move(barrierPenaltyConfig)};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
RelaxedBarrierPenalty::Config LeggedRobotInterface::loadFootPlacementSettings(const std::string& taskFile) const {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  const std::string prefix = "footPlancementSoftConstraint.";

  RelaxedBarrierPenalty::Config barrierPenaltyConfig;
  bool display = true;
  if (display) {
    std::cerr << "\n #### Foot Placement Cone Settings: ";
    std::cerr << "\n #### =============================================================================\n";
  }
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", display);
  loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", display);
  if (display) {
    std::cerr << " #### =============================================================================\n";
  }

  return std::move(barrierPenaltyConfig);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
                                                                                const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  FrictionConeConstraint::Config frictionConeConConfig(frictionCoefficient);
  std::unique_ptr<FrictionConeConstraint> frictionConeConstraintPtr(
      new FrictionConeConstraint(*referenceManagerPtr_, std::move(frictionConeConConfig), contactPointIndex, centroidalModelInfo_));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(frictionConeConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotInterface::getCBFFootPlacementConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                  const std::string& modelName, size_t contactPointIndex,
                                                                  const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  CBFFootPlacementConstraint::Config footplacementConstraintConfig;
  std::unique_ptr<CBFFootPlacementConstraint> footplacementConstraintPtr(
      new CBFFootPlacementConstraint(*referenceManagerPtr_, eeKinematics, centroidalModelInfo_, modelName,
                            std::move(footplacementConstraintConfig), contactPointIndex));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateInputCost>(new StateInputSoftConstraint(std::move(footplacementConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> LeggedRobotInterface::getStateOnlyFootPlacementConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                  const std::string& modelName, size_t contactPointIndex,
                                                                  const RelaxedBarrierPenalty::Config& barrierPenaltyConfig) {
  StateOnlyFootPlacementConstraint::Config footplacementConstraintConfig;
  std::unique_ptr<StateOnlyFootPlacementConstraint> footplacementConstraintPtr(
      new StateOnlyFootPlacementConstraint(*referenceManagerPtr_, eeKinematics, modelName,
                            std::move(footplacementConstraintConfig), contactPointIndex, centroidalModelInfo_.stateDim));

  std::unique_ptr<PenaltyBase> penalty(new RelaxedBarrierPenalty(barrierPenaltyConfig));

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(footplacementConstraintPtr), std::move(penalty)));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getZeroForceConstraint(size_t contactPointIndex) {
  return std::unique_ptr<StateInputConstraint>(new ZeroForceConstraint(*referenceManagerPtr_, contactPointIndex, centroidalModelInfo_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                      size_t contactPointIndex,
                                                                                      bool useAnalyticalGradients) {
  auto eeZeroVelConConfig = [](scalar_t positionErrorGain) {
    EndEffectorLinearConstraint::Config config;
    config.b.setZero(3);
    config.Av.setIdentity(3, 3);
    if (!numerics::almost_eq(positionErrorGain, 0.0)) {
      config.Ax.setZero(3, 3);
      config.Ax(2, 2) = positionErrorGain;
    }
    return config;
  };

  if (useAnalyticalGradients) {
    throw std::runtime_error(
        "[LeggedRobotInterface::getZeroVelocityConstraint] The analytical end-effector zero velocity constraint is not implemented!");
  } else {
    return std::unique_ptr<StateInputConstraint>(new ZeroVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex,
                                                                                 eeZeroVelConConfig(modelSettings_.positionErrorGain)));
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotInterface::getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                        size_t contactPointIndex,
                                                                                        bool useAnalyticalGradients) {
  if (useAnalyticalGradients) {
    throw std::runtime_error(
        "[LeggedRobotInterface::getNormalVelocityConstraint] The analytical end-effector normal velocity constraint is not implemented!");
  } else {
    return std::unique_ptr<StateInputConstraint>(new NormalVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex));
  }
}

}  // namespace legged_robot
}  // namespace ocs2
