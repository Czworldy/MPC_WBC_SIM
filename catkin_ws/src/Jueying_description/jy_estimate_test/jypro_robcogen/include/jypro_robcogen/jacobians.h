#ifndef JYPRO_JACOBIANS_H_
#define JYPRO_JACOBIANS_H_

#include <iit/rbd/TransformsBase.h>
#include "declarations.h"
#include "kinematics_parameters.h"
#include "transforms.h" // to use the same 'Parameters' struct defined there
#include "model_constants.h"

namespace JYPro {
namespace rcg {

template<int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<JointState, COLS, M>
{};

/**
 *
 */
class Jacobians
{
    public:
        
        struct Type_fr_BASE_J_fr_LF_FOOT : public JacobianT<3, Type_fr_BASE_J_fr_LF_FOOT>
        {
            Type_fr_BASE_J_fr_LF_FOOT();
            const Type_fr_BASE_J_fr_LF_FOOT& update(const JointState&);
        };
        
        
        struct Type_fr_BASE_J_fr_RF_FOOT : public JacobianT<3, Type_fr_BASE_J_fr_RF_FOOT>
        {
            Type_fr_BASE_J_fr_RF_FOOT();
            const Type_fr_BASE_J_fr_RF_FOOT& update(const JointState&);
        };
        
        
        struct Type_fr_BASE_J_fr_LH_FOOT : public JacobianT<3, Type_fr_BASE_J_fr_LH_FOOT>
        {
            Type_fr_BASE_J_fr_LH_FOOT();
            const Type_fr_BASE_J_fr_LH_FOOT& update(const JointState&);
        };
        
        
        struct Type_fr_BASE_J_fr_RH_FOOT : public JacobianT<3, Type_fr_BASE_J_fr_RH_FOOT>
        {
            Type_fr_BASE_J_fr_RH_FOOT();
            const Type_fr_BASE_J_fr_RH_FOOT& update(const JointState&);
        };
        
        
        struct Type_fr_imu_link_J_fr_LF_FOOT : public JacobianT<3, Type_fr_imu_link_J_fr_LF_FOOT>
        {
            Type_fr_imu_link_J_fr_LF_FOOT();
            const Type_fr_imu_link_J_fr_LF_FOOT& update(const JointState&);
        };
        
        
        struct Type_fr_imu_link_J_fr_RF_FOOT : public JacobianT<3, Type_fr_imu_link_J_fr_RF_FOOT>
        {
            Type_fr_imu_link_J_fr_RF_FOOT();
            const Type_fr_imu_link_J_fr_RF_FOOT& update(const JointState&);
        };
        
        
        struct Type_fr_imu_link_J_fr_LH_FOOT : public JacobianT<3, Type_fr_imu_link_J_fr_LH_FOOT>
        {
            Type_fr_imu_link_J_fr_LH_FOOT();
            const Type_fr_imu_link_J_fr_LH_FOOT& update(const JointState&);
        };
        
        
        struct Type_fr_imu_link_J_fr_RH_FOOT : public JacobianT<3, Type_fr_imu_link_J_fr_RH_FOOT>
        {
            Type_fr_imu_link_J_fr_RH_FOOT();
            const Type_fr_imu_link_J_fr_RH_FOOT& update(const JointState&);
        };
        
    public:
        Jacobians();
        void updateParameters(const Params_lengths& _lengths, const Params_angles& _angles);
    public:
        Type_fr_BASE_J_fr_LF_FOOT fr_BASE_J_fr_LF_FOOT;
        Type_fr_BASE_J_fr_RF_FOOT fr_BASE_J_fr_RF_FOOT;
        Type_fr_BASE_J_fr_LH_FOOT fr_BASE_J_fr_LH_FOOT;
        Type_fr_BASE_J_fr_RH_FOOT fr_BASE_J_fr_RH_FOOT;
        Type_fr_imu_link_J_fr_LF_FOOT fr_imu_link_J_fr_LF_FOOT;
        Type_fr_imu_link_J_fr_RF_FOOT fr_imu_link_J_fr_RF_FOOT;
        Type_fr_imu_link_J_fr_LH_FOOT fr_imu_link_J_fr_LH_FOOT;
        Type_fr_imu_link_J_fr_RH_FOOT fr_imu_link_J_fr_RH_FOOT;

    protected:
        Parameters params;

};


}
}

#endif
