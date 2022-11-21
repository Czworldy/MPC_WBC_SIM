#pragma once

#include "EndEffectorLinearConstraint.h"
#include "SwitchedModelReferenceManager.h"

#include <ocs2_core/constraint/StateInputConstraint.h>

namespace ocs2 {
namespace legged_robot {

/**
 * Specializes the CppAd version of zero velocity constraint on an end-effector position and linear velocity.
 * Constructs the member EndEffectorLinearConstraint object with number of constraints of 3.
 *
 * See also EndEffectorLinearConstraint for the underlying computation.
 */
class ZeroVelocityConstraintCppAd final : public StateInputConstraint {
    public:
        /**
         * Constructor
         * @param [in] referenceManager : Switched model ReferenceManager
         * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
         * @param [in] contactPointIndex : The 3 DoF contact index.
         * @param [in] config: The constraint coefficients
         */
        ZeroVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                    const EndEffectorKinematics<scalar_t>& endEffectorKinematics,  size_t contactPointIndex,
                                    EndEffectorLinearConstraint::Config config = EndEffectorLinearConstraint::Config());
        
        ~ZeroVelocityConstraintCppAd() override = default;
        ZeroVelocityConstraintCppAd* clone() const override { return new ZeroVelocityConstraintCppAd(*this); }

        bool isActive(scalar_t time) const override;
        size_t getNumConstraints(scalar_t time) const override { return 3; }
        vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
        VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                 const PreComputation& preComp) const override;

    private:
        ZeroVelocityConstraintCppAd(const ZeroVelocityConstraintCppAd& rhs);

        const SwitchedModelReferenceManager* referenceManagerPtr_;                                                       
        std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;
        const size_t contactPointIndex_;      
};

} // namespace legged_robot
} // namespace ocs2
