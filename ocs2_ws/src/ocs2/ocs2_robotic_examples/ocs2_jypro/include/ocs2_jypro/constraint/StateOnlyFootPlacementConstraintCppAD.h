#pragma once

#include "ocs2_jypro/common/Types.h"
#include "ocs2_jypro/synchronized_module/SwitchedModelReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_jypro/constraint/EndEffectorLinearConstraint.h"
#include <ocs2_core/constraint/StateConstraintCppAd.h>

#include "ocs2_jypro/common/ModelSettings.h"
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>



namespace ocs2 {
namespace legged_robot {

class StateOnlyFootPlacementConstraint final : public StateConstraintCppAd {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        struct Config {
        explicit Config(
                        scalar_t hessianDiagonalShiftParam = 1e-6)
            :hessianDiagonalShift(hessianDiagonalShiftParam) {
            assert(hessianDiagonalShift >= 0.0);
        }
        scalar_t hessianDiagonalShift;
    };
    StateOnlyFootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager, 
                                    const PinocchioEndEffectorKinematicsCppAd& endEffectorKinematics,
                                    const std::string& modelName,
                                    Config config, size_t contactPointIndex,
                                    const CentroidalModelInfo& info);

    ~StateOnlyFootPlacementConstraint() override = default;
    StateOnlyFootPlacementConstraint* clone() const override { return new StateOnlyFootPlacementConstraint(*this); }

    bool isActive(scalar_t time) const override;
    size_t getNumConstraints(scalar_t time) const override { return 6; };
    ad_vector_t constraintFunction(ad_scalar_t time, const ad_vector_t& state,
                                       const ad_vector_t& parameters) const override;
    vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                        const PreComputation& preComp) const override;
    VectorFunctionQuadraticApproximation getQuadraticApproximation(scalar_t time, const vector_t& state,
                                                                 const PreComputation& preComp) const override;

    
    private:
        StateOnlyFootPlacementConstraint(const StateOnlyFootPlacementConstraint& other) 
            : StateConstraintCppAd(other),
            referenceManagerPtr_(other.referenceManagerPtr_),
            endEffectorKinematicsPtr_(other.endEffectorKinematicsPtr_->clone()),
            config_(other.config_),
            contactPointIndex_(other.contactPointIndex_){std::cout << "StateOnlyFootPlacementConstraint copy constructor" << std::endl;}

        const SwitchedModelReferenceManager* referenceManagerPtr_;
        // std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;
        std::unique_ptr<PinocchioEndEffectorKinematicsCppAd> endEffectorKinematicsPtr_;


        const Config config_;
        const size_t contactPointIndex_;
        const CentroidalModelInfo info_;

        Eigen::Matrix<size_t, 6, 4> B;

};

} // namespace legged_robot
} // namespace ocs2