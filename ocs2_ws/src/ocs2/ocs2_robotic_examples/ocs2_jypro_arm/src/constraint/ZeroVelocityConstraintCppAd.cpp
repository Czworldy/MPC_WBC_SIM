#include "ZeroVelocityConstraintCppAd.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroVelocityConstraintCppAd::ZeroVelocityConstraintCppAd(const SwitchedModelReferenceManager& referenceManager,
                                                         const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                         size_t contactPointIndex, EndEffectorLinearConstraint::Config config)
        : StateInputConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(&referenceManager),
          eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 3, std::move(config))),
          contactPointIndex_(contactPointIndex) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroVelocityConstraintCppAd::ZeroVelocityConstraintCppAd(const ZeroVelocityConstraintCppAd& rhs)
        : StateInputConstraint(rhs),
          referenceManagerPtr_(rhs.referenceManagerPtr_),
          eeLinearConstraintPtr_(rhs.eeLinearConstraintPtr_->clone()),
          contactPointIndex_(rhs.contactPointIndex_) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ZeroVelocityConstraintCppAd::isActive(scalar_t time) const {
    return referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ZeroVelocityConstraintCppAd::getValue(scalar_t time, const vector_t& state, const vector_t& input, 
                                               const PreComputation& preComp) const {
    return eeLinearConstraintPtr_->getValue(time, state, input, preComp);
} 

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ZeroVelocityConstraintCppAd::getLinearApproximation(scalar_t time, const vector_t& state,
                                                                                      const vector_t& input,
                                                                                      const PreComputation& preComp) const {
    return eeLinearConstraintPtr_->getLinearApproximation(time, state, input, preComp);
}

} // namespace leggef_robot
} // namespace ocs2
