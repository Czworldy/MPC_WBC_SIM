#pragma once

// ocs2
#include <ocs2_centroidal_model/FactoryFunctions.h>
#include <ocs2_core/Types.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_robotic_tools/common/RobotInterface.h>
#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>
#include <ocs2_sqp/MultipleShootingSettings.h>


#include "ModelSettings.h"
#include "LeggedRobotWithArmInitializer.h"
#include "SwitchedModelReferenceManager.h"
#include "ManipulatorModelInfo.h"

/**
 * LeggedRobotInterface class
 * General interface for mpc implementation on the legged robot model
 */
namespace ocs2 {
namespace legged_robot {

class LeggedRobotWithArmInterface final : public RobotInterface {
    public:
        /**
         * Constructor
         * @param [in] taskFileFolderName: The name of the folder containing task file
         * @param [in] targetCommandFile: The path of the target command file
         * @param [in] urdfTree: Pointer to a URDF model tree
         */
        LeggedRobotWithArmInterface(const std::string& taskFile, const std::string& urdfFile,
                                                         const std::string& referenceFile);
        
        ~LeggedRobotWithArmInterface() override = default;

        const OptimalControlProblem& getOptimalControlProblem() const override { return *problemPtr_; }

        const ModelSettings& modelSettings() const { return modelSettings_; }
        const ddp::Settings& ddpSettings() const { return ddpSettings_; }
        const mpc::Settings& mpcSettings() const { return mpcSettings_; }
        const multiple_shooting::Settings& sqpSettings() const { return sqpSettings_; }
        const rollout::Settings& rolloutSettings() const { return rolloutSettings_; }

        const vector_t& getInitialState() const { return initialState_; }
        const RolloutBase& getRollout() const { return *rolloutPtr_; }
        PinocchioInterface& getPinocchioInterface() { return *pinocchioInterfacePtr_; }
        const CentroidalModelInfo& getCentroidalModelInfo() const { return centroidalModelInfo_; }
        std::shared_ptr<SwitchedModelReferenceManager> getSwitchedModelReferenceManagerPtr() const { return referenceManagerPtr_; }

        const LeggedRobotWithArmInitializer& getInitializer() const override { return *initializerPtr_; }
        std::shared_ptr<ReferenceManagerInterface> getReferenceManagerPtr() const override { return referenceManagerPtr_; }

    private:
        std::shared_ptr<GaitSchedule> loadGaitSchedule(const std::string& taskFile);
        void setupOptimalControlProblem(const std::string& taskFile, const std::string& urdfFile, 
                                        const std::string referenceFile, bool verbose);
        std::unique_ptr<StateInputCost> getBaseTrackingCost(const std::string& taskFile, const CentroidalModelInfo& info);
        std::unique_ptr<StateInputCost> getEEPositionCost(const std::string& taskFile,
                                                          const PinocchioInterface& pinocchioInterface,
                                                          const PinocchioStateInputMapping<ad_scalar_t>& mapping,
                                                          std::string endEffectorId,
                                                          size_t stateDim, size_t inputDim, const std::string& modelName,
                                                          const std::string& modelFolder, bool recompileLibraries,
                                                          bool verbose);
        void initializeInputCostWeight(const std::string& taskFile, const CentroidalModelInfo& info,  matrix_t& R);

        std::pair<scalar_t, RelaxedBarrierPenalty::Config> loadFrictionConeSettings(const std::string& taskFile) const;
        std::unique_ptr<StateInputCost> getFrictionConeConstraint(size_t contactPointIndex, scalar_t frictionCoefficient, 
                                                                  const RelaxedBarrierPenalty::Config& barrierPenaltyConfig);
        std::unique_ptr<StateInputConstraint> getZeroForceConstraint(size_t contactPointIndex);
        std::unique_ptr<StateInputConstraint> getZeroVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                        size_t contactPointIndex, bool useAnalyticalGradients);
        std::unique_ptr<StateInputConstraint> getNormalVelocityConstraint(const EndEffectorKinematics<scalar_t>& eeKinematics,
                                                                         size_t contactPointIndex, bool useAnalyticalGradients);
        
        std::unique_ptr<StateCost> getEndEffectorConstraint(const std::string& taskFile, const std::string& prefix,
                                                            const EndEffectorKinematics<scalar_t>& eeKinematics);

        bool display_;
        ModelSettings modelSettings_;
        ddp::Settings ddpSettings_;
        mpc::Settings mpcSettings_;
        multiple_shooting::Settings sqpSettings_;


        std::unique_ptr<PinocchioInterface> pinocchioInterfacePtr_;
        CentroidalModelInfo centroidalModelInfo_;

        std::unique_ptr<OptimalControlProblem> problemPtr_;
        std::shared_ptr<SwitchedModelReferenceManager> referenceManagerPtr_;

        rollout::Settings rolloutSettings_;
        std::unique_ptr<RolloutBase> rolloutPtr_;
        std::unique_ptr<LeggedRobotWithArmInitializer> initializerPtr_;
        EndEffectorTrackMode endEffectorTrackMode;

        vector_t initialState_;

};

} // namespace legged_robot
} // namespace ocs2