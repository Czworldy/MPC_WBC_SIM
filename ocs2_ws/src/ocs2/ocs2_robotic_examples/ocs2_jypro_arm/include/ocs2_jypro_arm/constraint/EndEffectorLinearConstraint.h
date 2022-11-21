#pragma once

#include <memory>

#include <ocs2_core/constraint/StateInputConstraint.h>

#include <ocs2_robotic_tools/end_effector/EndEffectorKinematics.h>

namespace ocs2 {
namespace legged_robot {

/**
 * Defines a linear constraint on an end-effector position (xee) and linear velocity (vee).
 * g(xee, vee) = Ax * xee + Av * vee + b
 * - For defining constraint of type g(xee), set Av to matrix_t(0, 0)
 * - For defining constraint of type g(vee), set Ax to matrix_t(0, 0)
 */
class EndEffectorLinearConstraint final : public StateInputConstraint {
    public:
        /**
         * Coefficients of the linear constraints of the form:
         * g(xee, vee) = Ax * xee + Av * vee + b
         */    
        struct Config {
            vector_t b;
            matrix_t Ax;
            matrix_t Av;
        };   

        /**
         * Constructor
         * @param [in] endEffectorKinematics: The kinematic interface to the target end-effector.
         * @param [in] numConstraints: The number of constraints {1, 2, 3}
         * @param [in] config: The constraint coefficients, g(xee, vee) = Ax * xee + Av * vee + b
         */
        EndEffectorLinearConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics, size_t numConstraints,
                                    Config config = Config());
        
        ~EndEffectorLinearConstraint() override = default;
        EndEffectorLinearConstraint* clone() const override { return new EndEffectorLinearConstraint(*this); }

        /** Sets a new constraint coefficients. */
        void configure(Config&& config);
        /** Sets a new constraint coefficients. */
        void configure(const Config& config) { this->configure(Config(config)); }

        /** Gets the underlying end-effector kinematics interface. */
        EndEffectorKinematics<scalar_t>& getEndEffectorKinematics() { return *endEffectorKinematicsPtr_; }

        size_t getNumConstraints(scalar_t time) const override { return numConstraints_; }
        vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;
        VectorFunctionLinearApproximation getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input, 
                                                                 const PreComputation& preComp) const override;

    private:
        EndEffectorLinearConstraint(const EndEffectorLinearConstraint& rhs);

        std::unique_ptr<EndEffectorKinematics<scalar_t>> endEffectorKinematicsPtr_;
        const size_t numConstraints_;
        Config config_;      
};

} // namespace legged_robot
} // namespace ocs2