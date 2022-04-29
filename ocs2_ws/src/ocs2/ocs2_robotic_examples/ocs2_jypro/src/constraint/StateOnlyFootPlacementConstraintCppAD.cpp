#include "ocs2_jypro/constraint/StateOnlyFootPlacementConstraintCppAD.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include "ocs2_jypro/LeggedRobotPreComputation.h"

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
StateOnlyFootPlacementConstraint::StateOnlyFootPlacementConstraint(const SwitchedModelReferenceManager& referenceManager,
                                                                   const EndEffectorKinematics<scalar_t>& endEffectorKinematics,
                                                                   Config config, size_t contactPointIndex,
                                                                   CentroidalModelInfo info)
    : StateInputConstraint(ConstraintOrder::Quadratic),
    referenceManagerPtr_(&referenceManager),
    eeLinearConstraintPtr_(new EndEffectorLinearConstraint(endEffectorKinematics, 6)),
    config_(std::move(config)),
    contactPointIndex_(contactPointIndex),
    info_(std::move(info)) {

        // eeLinearConstraintPtr_.reset(new EndEffectorLinearConstraint(endEffectorKinematics, 6, conf));
        size_t tor = 0.03;
        vector_t B_veclf = vector_t::Zero(6);
        vector_t B_vecrf = vector_t::Zero(6);
        vector_t B_veclh = vector_t::Zero(6);
        vector_t B_vecrh = vector_t::Zero(6);
        vector_t bias = tor * vector_t::Ones(6);
        B_veclf << 0.2, -0.2, -0.32, 0.32, 0.0, 0.0;
        B_vecrf << -0.15, 0.15, -0.36, 0.36, 0.0, 0.0;
        B_veclh << 0.15, -0.15, 0.31, -0.31, 0.0, 0.0;
        B_vecrh << -0.2, 0.2, 0.3, -0.3, 0.0, 0.0;

        B_veclf = (B_veclf + bias).eval(); 
        B_vecrf = (B_vecrf + bias).eval(); 
        B_veclh = (B_veclh + bias).eval(); 
        B_vecrh = (B_vecrh + bias).eval(); 

        B << B_veclf, B_vecrf, B_veclh, B_vecrh;
        
    }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool StateOnlyFootPlacementConstraint::isActive(scalar_t time) const {
  return !referenceManagerPtr_->getContactFlags(time)[contactPointIndex_];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t StateOnlyFootPlacementConstraint::getValue(scalar_t time, const vector_t& state, const vector_t& input,
                                          const PreComputation& preComp) const {
    const auto& preCompLegged = cast<LeggedRobotPreComputation>(preComp);
    EndEffectorLinearConstraint::Config conf;
    // conf.Av = matrix_t(0, 0);
    conf.Ax = matrix_t(6, 3);
    conf.Ax << 1, 0, 0,
                -1, 0, 0,
                0, 1, 0,
                0, -1, 0,
                0, 0, 1,
                0, 0, -1;
    conf.b = B.col(contactPointIndex_);
    eeLinearConstraintPtr_->configure(conf);
    eeLinearConstraintPtr_->getValue(time, state, input, preComp);
    preCompLegged.getSwingTimeLeft()[contactPointIndex_];
    
    

}

}
}