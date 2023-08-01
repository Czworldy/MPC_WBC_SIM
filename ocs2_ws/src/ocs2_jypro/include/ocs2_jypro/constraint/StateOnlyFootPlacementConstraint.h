#pragma once

#include "ocs2_jypro/common/Types.h"
#include "ocs2_jypro/synchronized_module/SwitchedModelReferenceManager.h"

#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <ocs2_core/constraint/StateInputConstraint.h>

#include "ocs2_jypro/constraint/EndEffectorLinearConstraint.h"
#include <ocs2_core/constraint/StateConstraint.h>

#include "ocs2_jypro/common/ModelSettings.h"
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematicsCppAd.h>

#include "ocs2_jypro/foot_planner/CubicSpline.h"



namespace ocs2 {
namespace legged_robot {

class StateOnlyFootPlacementConstraint  : public StateConstraint {
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
                                    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                    size_t contactPointIndex);


    ~StateOnlyFootPlacementConstraint() override = default;
    StateOnlyFootPlacementConstraint* clone() const override { return new StateOnlyFootPlacementConstraint(*referenceManagerPtr_, *endEffectorKinematicsPtr_, contactPointIndex_); }

    bool isActive(scalar_t time) const override;
    size_t getNumConstraints(scalar_t time) const override;
    vector_t getValue(scalar_t time, const vector_t& state, const PreComputation& preComp) const override;
    VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state,
                                                        const PreComputation& preComp) const override;


    private:
        StateOnlyFootPlacementConstraint(const StateOnlyFootPlacementConstraint& other) = default;


        const SwitchedModelReferenceManager* referenceManagerPtr_;
        // std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;
        PinocchioEndEffectorKinematicsCppAd* endEffectorKinematics_;
        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;

        const size_t contactPointIndex_;

        scalar_t tor = 0.05, stance_tol = 0.05;

        const CubicSpline transitionSpline_;

};

} // namespace legged_robot
} // namespace ocs2