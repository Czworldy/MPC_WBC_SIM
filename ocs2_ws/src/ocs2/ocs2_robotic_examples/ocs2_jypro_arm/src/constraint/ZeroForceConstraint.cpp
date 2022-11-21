#include "ZeroForceConstraint.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ZeroForceConstraint::ZeroForceConstraint(const SwitchedModelReferenceManager& referenceManager, size_t contactPointIndex,
                                         CentroidalModelInfo info)
        : StateInputConstraint(ConstraintOrder::Linear),
          referenceManagerPtr_(&referenceManager),
          contactPointIndex_(contactPointIndex),
          info_(std::move(info)) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool ZeroForceConstraint::isActive(scalar_t time) const {
    return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t ZeroForceConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const {
    return centroidal_model::getContactForces(input, contactPointIndex_, info_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
VectorFunctionLinearApproximation ZeroForceConstraint::getLinearApproximation(scalar_t time, const vector_t& state, const vector_t& input, 
                                                                              const PreComputation& preComp) const {
    VectorFunctionLinearApproximation approx;
    approx.f = getValue(time, state, input, preComp);
    approx.dfdx = matrix_t::Zero(3, state.size());
    approx.dfdu = matrix_t::Zero(3, input.size());
    approx.dfdu.middleCols<3>(3 * contactPointIndex_).diagonal() = vector_t::Ones(3);
    return approx;
}


} // namespace legged_robot
} // namespace ocs2