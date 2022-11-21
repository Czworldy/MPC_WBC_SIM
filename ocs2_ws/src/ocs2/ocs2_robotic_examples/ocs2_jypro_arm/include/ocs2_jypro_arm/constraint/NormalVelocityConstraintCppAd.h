#pragma once

#include "EndEffectorLinearConstraint.h"
#include "SwitchedModelReferenceManager.h"

#include <ocs2_core/constraint/StateInputConstraint.h>

namespace ocs2 {
namespace legged_robot {

/**
 * Specializes the CppAd version of normal velocity constraint on an end-effector position and linear velocity.
 * Constructs the member EndEffectorLinearConstraint object with number of constraints of 1.
 *
 * See also EndEffectorLinearConstraint for the underlying computation.
 */
class NormalVelocityConstraintCppAd final : public StateInputConstraint {
    public:
        /**
         * Constructor
         * @param [in] referenceManager : Switched model ReferenceManager
         * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
         * @param [in] contactPointIndex : The 3 DoF contact index.
         */    
        NormalVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                  const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t contactPointIndex);
        ~NormalVelocityConstraintCppAd() override = default;
        NormalVelocityConstraintCppAd* clone() const override { return new NormalVelocityConstraintCppAd(*this); }

        bool isActive(scalar_t time) const override;
        size_t getNumConstraints(scalar_t time) const override { return 1; }
        vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
        VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input,
                                                                    const PreComputation& preComp) const override;

    private:
        NormalVelocityConstraintCppAd(const NormalVelocityConstraintCppAd& rhs);

        const SwitchedModelReferenceManager* referenceManagerPtr_;
        std::unique_ptr<EndEffectorLinearConstraint> eeLinearConstraintPtr_;
        const size_t contactPointIndex_;
};

} // namespace legged_robot
} // namespace ocs2