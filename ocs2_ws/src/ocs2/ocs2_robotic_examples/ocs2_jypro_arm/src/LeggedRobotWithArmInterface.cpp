#include <iostream>
#include <string>

#include <pinocchio/fwd.hpp> // forward declarations must be included first.

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "LeggedRobotWithArmInterface.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>
#include <ocs2_core/misc/Display.h>
#include <ocs2_core/soft_constraint/StateInputSoftConstraint.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_core/soft_constraint/StateSoftConstraint.h>

#include "LeggedRobotWithArmPreComputation.h"
#include "FrictionConeConstraint.h"
#include "NormalVelocityConstraintCppAd.h"
#include "ZeroForceConstraint.h"
#include "ZeroVelocityConstraintCppAd.h"
#include "LeggedRobotWithArmStateInputQuadraticCost.h"
#include "QuadraticEndEffectorPositionCostCppAd.h"
#include "LeggedRobotWithArmDynamicsAD.h"
#include "EndEffectorConstraint.h"

// Boost
#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/path.hpp>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotWithArmInterface::LeggedRobotWithArmInterface(const std::string& taskFile, const std::string& urdfFile,
                                                         const std::string& referenceFile) {
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

    // Load the task file
    std::cerr << "Loading task file: " << taskFile << std::endl;

    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    loadData::loadPtreeValue(pt, display_, "legged_robot_interface.display", true);
    const bool verbose = display_;

    // Load setting from loading file
    modelSettings_ = loadModelSettings(taskFile, "model_settings", verbose);
    ddpSettings_ = ddp::loadSettings(taskFile, "ddp", verbose);
    mpcSettings_ = mpc::loadSettings(taskFile, "mpc", verbose);
    rolloutSettings_ = rollout::loadSettings(taskFile, "rollout", verbose);
    sqpSettings_ = multiple_shooting::loadSettings(taskFile, "multiple_shooting", verbose);

    // read endEffectorTrackMode type
    endEffectorTrackMode = loadEndEffectorTrackMode(taskFile);
    std::cerr << "\n #### End Effector Tracking Information:";
    std::cerr << "\n #### =============================================================================\n";
    std::cerr << "\n #### endEffectorTrackMode: " << static_cast<int>(endEffectorTrackMode);

    // OptimalControlProblem
    setupOptimalControlProblem(taskFile, urdfFile, referenceFile, verbose);

    std::cerr << "[LeggedRobotWithArmInterface] construction done! dqwang================================" << std::endl;

    // Initial state
    initialState_.setZero(centroidalModelInfo_.stateDim);
    loadData::loadEigenMatrix(taskFile, "initialState", initialState_);

    std::cout << "initialState: \n" << initialState_ << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::shared_ptr<GaitSchedule> LeggedRobotWithArmInterface::loadGaitSchedule(const std::string& taskFile) {
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
void LeggedRobotWithArmInterface::setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, 
                                                             const std::string referenceFile, bool verbose) {
    // PinocchioInterface
    pinocchioInterfacePtr_.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNames)));

    // CentroidalModelInfo
    centroidalModelInfo_ = centroidal_model::createCentroidalModelInfo(
            *pinocchioInterfacePtr_, centroidal_model::loadCentroidalType(taskFile), 
            centroidal_model::loadDefaultJointState(18, referenceFile), modelSettings_.contactNames3DoF, modelSettings_.contactNames6DoF);
    
    // Swing trajectory planner
    std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(
            new SwingTrajectoryPlanner(loadSwingTrajectorySettings(taskFile, "swing_trajectory_config"), 4));

    // Mode schedule manager
    referenceManagerPtr_ = std::make_shared<SwitchedModelReferenceManager>(loadGaitSchedule(taskFile), std::move(swingTrajectoryPlanner));

    // Optimal control problem
    problemPtr_.reset(new OptimalControlProblem);

    // Dynamics
    bool useAnalyticalGradientsDynamics = false;
    loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsDynamics", useAnalyticalGradientsDynamics);
    std::unique_ptr<SystemDynamicsBase> dynamicsPtr;
    if (useAnalyticalGradientsDynamics) {
        throw std::runtime_error("[LeggedRobotWithArmInterface::setupOptimalControlProblem] The analytical dynamics class is not yet implemented.");
    } else {
        const std::string modelName = "dynamics";
        dynamicsPtr.reset(new LeggedRobotWithArmDynamicsAD(*pinocchioInterfacePtr_, centroidalModelInfo_, modelName, modelSettings_));
    }

    problemPtr_->dynamicsPtr = std::move(dynamicsPtr);

    // Cost terms
    problemPtr_->costPtr->add("baseTrackingCost", getBaseTrackingCost(taskFile, centroidalModelInfo_));

    const auto infoCppAd = centroidalModelInfo_.toCppAd();
    const CentroidalModelPinocchioMappingCppAd pinocchioMappingCppAd(infoCppAd);
    // problemPtr_->costPtr->add("endEffectorPositionCost", getEEPositionCost(taskFile, *pinocchioInterfacePtr_, pinocchioMappingCppAd, modelSettings_.contactNames3DoF[4],
    //                                                                        centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,modelSettings_.contactNames3DoF[4],
    //                                                                        modelSettings_.modelFolderCppAd,modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));

    // Constraint terms
    // friction cone settings
    scalar_t frictionCoefficient = 0.7;
    RelaxedBarrierPenalty::Config barrierPenaltyConfig;
    std::tie(frictionCoefficient, barrierPenaltyConfig) = loadFrictionConeSettings(taskFile);

    bool useAnalyticalGradientsConstraints = false;
    loadData::loadCppDataType(taskFile, "legged_robot_interface.useAnalyticalGradientsConstraints", useAnalyticalGradientsConstraints);
    for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++) { 
        const std::string& limbName = modelSettings_.contactNames3DoF[i];

        std::unique_ptr<EndEffectorKinematics<scalar_t>> eeKinematicsPtr;
        if (useAnalyticalGradientsConstraints) {
            throw std::runtime_error(
                    "[LeggedRobotWithArmInterface::setupOptimalControlProblem] The analytical end-effector linear contraint is not implemented!");       
        } else {
            auto velocityUpdateCallback = [&infoCppAd](const ad_vector_t& state, PinocchioInterfaceCppAd& pinocchioInterfaceAd) {
                const ad_vector_t q = centroidal_model::getGeneralizedCoordinates(state, infoCppAd);
                updateCentroidalDynamics(pinocchioInterfaceAd, infoCppAd, q);
            };
            eeKinematicsPtr.reset(new PinocchioEndEffectorKinematicsCppAd(*pinocchioInterfacePtr_, pinocchioMappingCppAd, {limbName},
                                                                          centroidalModelInfo_.stateDim, centroidalModelInfo_.inputDim,
                                                                          velocityUpdateCallback, limbName, modelSettings_.modelFolderCppAd,
                                                                          modelSettings_.recompileLibrariesCppAd, modelSettings_.verboseCppAd));
        }

        if (i < 4) {
            // Foot constraints
            problemPtr_->softConstraintPtr->add(limbName + "_frictionCone",
                                                getFrictionConeConstraint(i, frictionCoefficient, barrierPenaltyConfig));
            problemPtr_->equalityConstraintPtr->add(limbName + "_zeroForce", getZeroForceConstraint(i));
            problemPtr_->equalityConstraintPtr->add(limbName + "_zeroVelocity",
                                                    getZeroVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));
            problemPtr_->equalityConstraintPtr->add(limbName + "_normalVelocity",
                                                getNormalVelocityConstraint(*eeKinematicsPtr, i, useAnalyticalGradientsConstraints));
        }

        if (i == 4) {
            // Arm constraints
            size_t gripperIndex(4);
            problemPtr_->equalityConstraintPtr->add(modelSettings_.contactNames3DoF[gripperIndex] + "_zeroForce", getZeroForceConstraint(gripperIndex));

            std::cout << "endEffectorTrackMode11: " << int(endEffectorTrackMode) << std::endl;
            if(endEffectorTrackMode == EndEffectorTrackMode::EESoftConstraint) {
                // end-effector state constraint
                problemPtr_->stateSoftConstraintPtr->add("endEffector", getEndEffectorConstraint(taskFile, "endEffector", *eeKinematicsPtr));
                problemPtr_->finalSoftConstraintPtr->add("finalEndEffector", getEndEffectorConstraint(taskFile, "finalEndEffector", *eeKinematicsPtr));
                std::cout << "endEffectorTrackMode22: " << int(endEffectorTrackMode) << std::endl;
            }
        }
    }



    // Pre-computation
    problemPtr_->preComputationPtr.reset(new LeggedRobotWithArmPreComputation(*pinocchioInterfacePtr_, centroidalModelInfo_, 
                                                                              *referenceManagerPtr_->getSwingTrajectoryPlanner(), modelSettings_));
    // Rollout
    rolloutPtr_.reset(new TimeTriggeredRollout(*problemPtr_->dynamicsPtr, rolloutSettings_));

    // Initialization
    constexpr bool extendNormalizedMomentum = true;
    initializerPtr_.reset(new LeggedRobotWithArmInitializer(centroidalModelInfo_, *referenceManagerPtr_, extendNormalizedMomentum));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotWithArmInterface::initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info, matrix_t& R) {
   vector_t initialState(centroidalModelInfo_.stateDim);
   loadData::loadEigenMatrix(taskFile, "initialState", initialState); 

   std::cerr << "[LeggedRobotWithArmInterface::initializeInputCostWeight] ER_DONE!=========================DQWANG========================\n";

   const auto& model = pinocchioInterfacePtr_->getModel();
   auto& data = pinocchioInterfacePtr_->getData();
   const auto q = centroidal_model::getGeneralizedCoordinates(initialState, centroidalModelInfo_);
   pinocchio::computeJointJacobians(model, data, q);
   pinocchio::updateFramePlacements(model, data);

   matrix_t baseToLimbsJacobians(3 * info.numThreeDofContacts, 15);
   for (size_t i = 0; i < info.numThreeDofContacts; i++) {
       matrix_t jacobianWorldToContactPointInWorldFrame = matrix_t::Zero(6, info.generalizedCoordinatesNum);
       pinocchio::getFrameJacobian(model, data, model.getBodyId(modelSettings_.contactNames3DoF[i]), pinocchio::LOCAL_WORLD_ALIGNED,
                                   jacobianWorldToContactPointInWorldFrame);

        baseToLimbsJacobians.block(3 * i, 0, 3, 15) = (jacobianWorldToContactPointInWorldFrame.topRows<3>()).block(0, 6, 3, 15);
   }

   const size_t totalContactDim = 3 * info.numThreeDofContacts;
   R.block(totalContactDim, totalContactDim, 15, 15) = 
            // (baseToLimbsJacobians.transpose() * R.block(totalContactDim, totalContactDim, 16, 16) * baseToLimbsJacobians).eval();
               (baseToLimbsJacobians.transpose() * R.block(totalContactDim, totalContactDim, 15, 15) * baseToLimbsJacobians).eval();

    std::cerr << "[LeggedRobotWithArmInterface::initializeInputCostWeight] DONE!=========================DQWANG========================\n";
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotWithArmInterface::getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info) {
    matrix_t Q(info.stateDim, info.stateDim);
    loadData::loadEigenMatrix(taskFile, "Q", Q);
    matrix_t R(info.inputDim, info.inputDim);
    loadData::loadEigenMatrix(taskFile, "R", R);

    initializeInputCostWeight(taskFile, info, R);

    if (display_) {
        std::cerr << "\n #### Base Tracking Cost Coefficients: ";
        std::cerr << "\n #### =================================================================================\n";
        std::cerr << "Q:\n" << Q << "\n";
        std::cerr << "R:\n" << R << "\n";
        std::cerr << " #### ===================================================================================\n";
    }

    return std::unique_ptr<StateInputCost>(new LeggedRobotWithArmStateInputQuadraticCost(std::move(Q), std::move(R), info, *referenceManagerPtr_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotWithArmInterface::getEEPositionCost(const std::string& taskFile,
                                                                               const PinocchioInterface& pinocchioInterface,
                                                                               const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                                               std::string endEffectorId,
                                                                               size_t stateDim, size_t inputDim, const std::string& modelName,
                                                                               const std::string& modelFolder, bool recompileLibraries,
                                                                               bool verbose){
    matrix_t Q(3, 3);
    loadData::loadEigenMatrix(taskFile, "Q_eePosition", Q); 
    if (display_) {
        std::cerr << "\n #### End-effector Position Cost Coefficients: ";
        std::cerr << "\n #### =================================================================================\n";
        std::cerr << "Q:\n" << Q << "\n";
        std::cerr << " #### ===================================================================================\n";
    }

    std::cerr << "[LeggedRobotWithArmInterface::getEEPositionCost]: DONE!===============================================================================================================================================================================================================================================================================================================================================\n";     

    return std::unique_ptr<StateInputCost>(new QuadraticEndEffectorPositionCostCppAd(Q, pinocchioInterface, mapping, endEffectorId, stateDim, inputDim, modelName, modelFolder, recompileLibraries, verbose));

    
                                                                             
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::pair<scalar_t, RelaxedBarrierPenalty::Config> LeggedRobotWithArmInterface::loadFrictionConeSettings(const std::string& taskFile) const {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(taskFile, pt);
    const std::string prefix = "frictionConeSoftConstraint.";

    scalar_t frictionCoefficient = 1.0;
    RelaxedBarrierPenalty::Config barrierPenaltyConfig;
    if (display_) {
        std::cerr << "\n #### Friction Cone Settings: ";
        std::cerr << "\n #### ====================================================================================\n";
    }
    loadData::loadPtreeValue(pt, frictionCoefficient, prefix + "frictionCoefficient", display_);
    loadData::loadPtreeValue(pt, barrierPenaltyConfig.mu, prefix + "mu", display_);
    loadData::loadPtreeValue(pt, barrierPenaltyConfig.delta, prefix + "delta", display_);
    if(display_) {
        std::cerr << " #### =======================================================================================\n";
    }

    return {frictionCoefficient, std::move(barrierPenaltyConfig)};
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputCost> LeggedRobotWithArmInterface::getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient,
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
std::unique_ptr<StateInputConstraint> LeggedRobotWithArmInterface::getZeroForceConstraint(size_t contactPointIndex) {
    return std::unique_ptr<StateInputConstraint>(new ZeroForceConstraint(*referenceManagerPtr_, contactPointIndex, centroidalModelInfo_));
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotWithArmInterface::getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                             size_t contactPointIndex, 
                                                                                             bool useAnalyticalGradients) {
    auto eeZeroVelConConfig = [](scalar_t positionErrorGain) {
        EndEffectorLinearConstraint::Config config;
        config.b.setZero(3);
        config.Av.setIdentity(3, 3);
        if  (!numerics::almost_eq(positionErrorGain, 0.0)) {
            config.Ax.setZero(3, 3);
            config.Ax(2, 2) = positionErrorGain;
        }
        return config;
    };

    if (useAnalyticalGradients) {
        throw std::runtime_error(
                "[LeggedRobotWithArmInterface::getZeroVelocityConstraint] The analytical end-effector zero velocity constraint is not implemented!");
    } else {
        return std::unique_ptr<StateInputConstraint>(new ZeroVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex, 
                                                                                     eeZeroVelConConfig(modelSettings_.positionErrorGain)));
    }                                                                                               
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateInputConstraint> LeggedRobotWithArmInterface::getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                                               size_t contactPointIndex,
                                                                                               bool useAnalyticalGradients) {
    if (useAnalyticalGradients) {
        throw std::runtime_error(
                "[LeggedRobotWithArmInterface::getNormalVelocityConstraint] The analytical end-effector normal velocity constraint is not implemented!");
    } else {
        return std::unique_ptr<StateInputConstraint>(new NormalVelocityConstraintCppAd(*referenceManagerPtr_, eeKinematics, contactPointIndex));
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::unique_ptr<StateCost> LeggedRobotWithArmInterface::getEndEffectorConstraint(const std::string& taskFile, const std::string& prefix,
                                                                                 const EndEffectorKinematics<scalar_t>& eeKinematics) {
  scalar_t muPosition = 1.0;
  scalar_t muOrientation = 1.0;
  const std::string name = "WRIST_2";

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(taskFile, pt);
  std::cerr << "\n #### " << prefix << " Settings: ";
  std::cerr << "\n #### =============================================================================\n";
  loadData::loadPtreeValue(pt, muPosition, prefix + ".muPosition", true);
  loadData::loadPtreeValue(pt, muOrientation, prefix + ".muOrientation", true);
  std::cerr << " #### =============================================================================\n";

  if (referenceManagerPtr_ == nullptr) {
    throw std::runtime_error("[getEndEffectorConstraint] referenceManagerPtr_ should be set first!");
  }

  std::unique_ptr<StateConstraint> constraint;

  constraint.reset(new EndEffectorConstraint(eeKinematics, *referenceManagerPtr_));

  std::vector<std::unique_ptr<PenaltyBase>> penaltyArray(6);
  std::generate_n(penaltyArray.begin(), 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muPosition)); });
  std::generate_n(penaltyArray.begin() + 3, 3, [&] { return std::unique_ptr<PenaltyBase>(new QuadraticPenalty(muOrientation)); });

  return std::unique_ptr<StateCost>(new StateSoftConstraint(std::move(constraint), std::move(penaltyArray)));
}



} // namespace legged_robot
} // namespace ocs2
