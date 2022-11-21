#include "EndEffectorLinearConstraint.h"
#include <iostream>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorLinearConstraint::EndEffectorLinearConstraint(const EndEffectorKinematics<scalar_t>& endEffectorKinematics, 
                                                         size_t numConstraints, Config config)
        : StateInputConstraint(ConstraintOrder::Linear),
          endEffectorKinematicsPtr_(endEffectorKinematics.clone()),
          numConstraints_(numConstraints),
          config_(std::move(config)) {
    if(endEffectorKinematicsPtr_->getIds().size() != 1) {
        throw std::runtime_error("[EndEffectorLinearConstraint] this class only accepts a single end-effector!");
    }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
EndEffectorLinearConstraint::EndEffectorLinearConstraint(const EndEffectorLinearConstraint& rhs)
        : StateInputConstraint(rhs),
          endEffectorKinematicsPtr_(rhs.endEffectorKinematicsPtr_->clone()),
          numConstraints_(rhs.numConstraints_),
          config_(rhs.config_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void EndEffectorLinearConstraint::configure(Config&& config) {
    assert(config.b.rows() == numConstraints_);
    assert(config.Ax.size() > 0 || config.Av.size());
    assert(config.Ax.size() > 0 && config.Ax.rows() == numConstraints_);
    assert(config.Ax.size() > 0 && config.Ax.cols() == 3);
    assert(config.Av.size() > 0 && config.Av.rows() == numConstraints_);
    assert(config.Av.size() > 0 && config.Av.cols() == 3);
    config_ = std::move(config);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t EndEffectorLinearConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, 
                                               const PreComputation& preComp) const {
    vector_t f = config_.b;
    if (config_.Ax.size() > 0) {
        f.noalias() += config_.Ax * endEffectorKinematicsPtr_->getPosition(state).front();
    }
    if(config_.Av.size() > 0) {
        f.noalias() += config_.Av * endEffectorKinematicsPtr_->getVelocity(state,input).front();
    }
    return f;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation EndEffectorLinearConstraint::getLinearApproximation(scalar_t time, const vector_t& state, 
                                                                                      const vector_t& input, 
                                                                                      const PreComputation& preComp) const {
    VectorFunctionLinearApproximation linearApproximation = 
        VectorFunctionLinearApproximation::Zero(getNumConstraints(time), state.size(), input.size());
    
    linearApproximation.f = config_.b;

    if(config_.Ax.size() > 0) {
        const auto positionApprox = endEffectorKinematicsPtr_->getPositionLinearApproximation(state).front();
        linearApproximation.f.noalias() += config_.Ax * positionApprox.f;
        linearApproximation.dfdx.noalias() += config_.Ax * positionApprox.dfdx;
    }

    if(config_.Av.size() > 0) {
        const auto velocityApprox = endEffectorKinematicsPtr_->getVelocityLinearApproximation(state, input).front();
        linearApproximation.f.noalias() += config_.Av * velocityApprox.f;
        linearApproximation.dfdx.noalias() += config_.Av * velocityApprox.dfdx;
        linearApproximation.dfdu.noalias() += config_.Av * velocityApprox.dfdu;
        //std::cout << "[dqwang__EndEffectorLinearConstraint::getLinearApproximation] :\n" << linearApproximation.dfdu << "\nDONE" << std::endl;
    }

    return linearApproximation;
}

} // namespace legged_robot
} // namespace ocs2