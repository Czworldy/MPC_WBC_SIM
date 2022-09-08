#include "pronto_x20_commons/feet_jacobians.hpp"

using namespace pronto::quadruped;
using namespace iit::rbd;

namespace X20 {
namespace rcg {


FootJac FeetJacobians::getFootJacobian(const JointState& q,
                                               const LegID& leg)
{
    switch(leg){
    case LF:
        return getFootJacobianLF(q);
    case RF:
        return getFootJacobianRF(q);
    case LH:
        return getFootJacobianLH(q);
    case RH:
        return getFootJacobianRH(q);
    default:
        return Matrix33d::Identity();
    }
}

FootJac FeetJacobians::getFootJacobianLF(const JointState& q){
    return jacs_.fr_BASE_J_fr_LF_FOOT(q).block<3,3>(LX,0);
}
FootJac FeetJacobians::getFootJacobianRF(const JointState& q){
    return jacs_.fr_BASE_J_fr_RF_FOOT(q).block<3,3>(LX,0);
}
FootJac FeetJacobians::getFootJacobianLH(const JointState& q){
    return jacs_.fr_BASE_J_fr_LH_FOOT(q).block<3,3>(LX,0);
}
FootJac FeetJacobians::getFootJacobianRH(const JointState& q){
    return jacs_.fr_BASE_J_fr_RH_FOOT(q).block<3,3>(LX,0);
}
FootJac FeetJacobians::getFootJacobian(const JointState &q,
                                               const LegID &leg,
                                               const double& foot_x,
                                               const double& foot_y){

    return getFootJacobian(q,leg);
}

}
}



