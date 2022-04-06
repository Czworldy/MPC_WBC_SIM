#pragma once

#include <pronto_quadruped_commons/feet_contact_forces.h>
#include <jypro_robcogen/inverse_dynamics.h>
#include <jypro_robcogen/jsim.h>
#include <pronto_jypro_commons/feet_jacobians.hpp>
#include <iit/rbd/rbd.h>
#include <pronto_quadruped_commons/declarations.h>
#include <pronto_quadruped_commons/leg_data_map.h>
#include <pronto_quadruped_commons/leg_vector_map.h>


namespace JYPro{
namespace rcg{

class FeetContactForces : public pronto::quadruped::FeetContactForces {
    
    public:

        // typedef pronto::quadruped::Vector3d Vector3d;
    // typedef pronto::quadruped::Vector3d Vector3d;
    typedef iit::rbd::Vector3d Vector3d;
    typedef pronto::quadruped::JointState JointState;
    typedef pronto::quadruped::LegID LegID;
    // using LegVectorMap = pronto::quadruped::LegDataMap<Vector3d>;
    typedef pronto::quadruped::LegDataMap<Vector3d> LegVectorMap;
            inline FeetContactForces() :
        inverse_dynamics_(inertia_prop_, motion_transf_),
        jsim_(inertia_prop_, force_transf_)
    {
    }

    

        inline pronto::quadruped::Vector3d getFootGRF(const pronto::quadruped::JointState& q,
                            const pronto::quadruped::JointState& qd,
                            const pronto::quadruped::JointState& tau,
                            const Quaterniond& orient,
                            const pronto::quadruped::LegID& leg,
                            const pronto::quadruped::JointState& qdd = pronto::quadruped::JointState::Zero(),
                            const pronto::quadruped::Vector3d& xd = pronto::quadruped::Vector3d::Zero(),
                            const pronto::quadruped::Vector3d& xdd = pronto::quadruped::Vector3d::Zero(),
                            const pronto::quadruped::Vector3d& omega = pronto::quadruped::Vector3d::Zero(),
                            const pronto::quadruped::Vector3d& omegad = pronto::quadruped::Vector3d::Zero()) {
            pronto::quadruped::Vector3d res;
            getFootGRF(q, qd, tau, orient, leg, res, qdd, xd, xdd, omega, omegad);
            return res;
        }

        bool getFootGRF(const pronto::quadruped::JointState& q,
                        const pronto::quadruped::JointState& qd,
                        const pronto::quadruped::JointState& tau,
                        const Quaterniond& orient,
                        const pronto::quadruped::LegID& leg,
                        pronto::quadruped::Vector3d& foot_grf,
                        const pronto::quadruped::JointState& qdd = pronto::quadruped::JointState::Zero(),
                        const pronto::quadruped::Vector3d& xd = pronto::quadruped::Vector3d::Zero(),
                        const pronto::quadruped::Vector3d& xdd = pronto::quadruped::Vector3d::Zero(),
                        const pronto::quadruped::Vector3d& omega = pronto::quadruped::Vector3d::Zero(),
                        const pronto::quadruped::Vector3d& omegad = pronto::quadruped::Vector3d::Zero());

        inline bool getFeetGRF(const pronto::quadruped::JointState& q,
                        const pronto::quadruped::JointState& qd,
                        const pronto::quadruped::JointState& tau,
                        const Quaterniond& orient,
                        pronto::quadruped::LegVectorMap& feet_grf,
                        const pronto::quadruped::JointState& qdd = pronto::quadruped::JointState::Zero(),
                        const pronto::quadruped::Vector3d& xd = pronto::quadruped::Vector3d::Zero(),
                        const pronto::quadruped::Vector3d& xdd = pronto::quadruped::Vector3d::Zero(),
                        const pronto::quadruped::Vector3d& omega = pronto::quadruped::Vector3d::Zero(),
                        const pronto::quadruped::Vector3d& omegad = pronto::quadruped::Vector3d::Zero()) {
            bool res_lf = getFootGRF(q, qd, tau, orient, pronto::quadruped::LegID::LF, feet_grf[pronto::quadruped::LegID::LF], qdd, xd, xdd, omega, omegad);
            bool res_rf = getFootGRF(q, qd, tau, orient, pronto::quadruped::LegID::RF, feet_grf[pronto::quadruped::LegID::RF], qdd, xd, xdd, omega, omegad);
            bool res_lh = getFootGRF(q, qd, tau, orient, pronto::quadruped::LegID::LH, feet_grf[pronto::quadruped::LegID::LH], qdd, xd, xdd, omega, omegad);
            bool res_rh = getFootGRF(q, qd, tau, orient, pronto::quadruped::LegID::RH, feet_grf[pronto::quadruped::LegID::RH], qdd, xd, xdd, omega, omegad);

            return(res_lf && res_rf && res_lh && res_rh);
        }

        pronto::quadruped::LegVectorMap getFeetGRF(const pronto::quadruped::JointState& q,
                                const pronto::quadruped::JointState& qd,
                                const pronto::quadruped::JointState& tau,
                                const Quaterniond& orient,
                                const pronto::quadruped::JointState& qdd = pronto::quadruped::JointState::Zero(),
                                const pronto::quadruped::Vector3d& xd = pronto::quadruped::Vector3d::Zero(),
                                const pronto::quadruped::Vector3d& xdd = pronto::quadruped::Vector3d::Zero(),
                                const pronto::quadruped::Vector3d& omega = pronto::quadruped::Vector3d::Zero(),
                                const pronto::quadruped::Vector3d& omegad = pronto::quadruped::Vector3d::Zero()) {
            pronto::quadruped::LegVectorMap res;
            getFeetGRF(q, qd, tau, orient, res, qdd, xd, xdd, omega, omegad);
            return res;
        }

        inline void setContactPoint(pronto::quadruped::LegID leg, double foot_x, double foot_y){

        }



// namespace pronto {
// namespace jypro {

// class FeetContactForces : public pronto::quadruped::FeetContactForces {

// public:
//     // typedef pronto::quadruped::Vector3d Vector3d;
//     // typedef pronto::quadruped::Vector3d Vector3d;
//     typedef iit::rbd::Vector3d Vector3d;
//     typedef pronto::quadruped::JointState JointState;
//     typedef pronto::quadruped::LegID LegID;
//     // using LegVectorMap = pronto::quadruped::LegDataMap<Vector3d>;
//     typedef pronto::quadruped::LegDataMap<Vector3d> LegVectorMap;

// public:
//     inline FeetContactForces() :
//         inverse_dynamics_(inertia_prop_, motion_transf_),
//         jsim_(inertia_prop_, force_transf_)
//     {
//     }

//     ~FeetContactForces() {}

//     inline Vector3d getFootGRF(const JointState& q,
//                         const JointState& qd,
//                         const JointState& tau,
//                         const Quaterniond& orient,
//                         const LegID& leg,
//                         const JointState& qdd = JointState::Zero(),
//                         const Vector3d& xd = Vector3d::Zero(),
//                         const Vector3d& xdd = Vector3d::Zero(),
//                         const Vector3d& omega = Vector3d::Zero(),
//                         const Vector3d& omegad = Vector3d::Zero()) {
//         Vector3d res;
//         getFootGRF(q, qd, tau, orient, leg, res, qdd, xd, xdd, omega, omegad);
//         return res;
//     }

//     bool getFootGRF(const JointState& q,
//                     const JointState& qd,
//                     const JointState& tau,
//                     const Quaterniond& orient,
//                     const LegID& leg,
//                     Vector3d& foot_grf,
//                     const JointState& qdd = JointState::Zero(),
//                     const Vector3d& xd = Vector3d::Zero(),
//                     const Vector3d& xdd = Vector3d::Zero(),
//                     const Vector3d& omega = Vector3d::Zero(),
//                     const Vector3d& omegad = Vector3d::Zero());

//     inline bool getFeetGRF(const JointState& q,
//                     const JointState& qd,
//                     const JointState& tau,
//                     const Quaterniond& orient,
//                     LegVectorMap& feet_grf,
//                     const JointState& qdd = JointState::Zero(),
//                     const Vector3d& xd = Vector3d::Zero(),
//                     const Vector3d& xdd = Vector3d::Zero(),
//                     const Vector3d& omega = Vector3d::Zero(),
//                     const Vector3d& omegad = Vector3d::Zero()) {
//         bool res_lf = getFootGRF(q, qd, tau, orient, LegID::LF, feet_grf[LegID::LF], qdd, xd, xdd, omega, omegad);
//         bool res_rf = getFootGRF(q, qd, tau, orient, LegID::RF, feet_grf[LegID::RF], qdd, xd, xdd, omega, omegad);
//         bool res_lh = getFootGRF(q, qd, tau, orient, LegID::LH, feet_grf[LegID::LH], qdd, xd, xdd, omega, omegad);
//         bool res_rh = getFootGRF(q, qd, tau, orient, LegID::RH, feet_grf[LegID::RH], qdd, xd, xdd, omega, omegad);

//         return(res_lf && res_rf && res_lh && res_rh);
//     }


//     pronto::quadruped::LegVectorMap getFeetGRF(const JointState& q,
//                             const JointState& qd,
//                             const JointState& tau,
//                             const Quaterniond& orient,
//                             const JointState& qdd = JointState::Zero(),
//                             const Vector3d& xd = Vector3d::Zero(),
//                             const Vector3d& xdd = Vector3d::Zero(),
//                             const Vector3d& omega = Vector3d::Zero(),
//                             const Vector3d& omegad = Vector3d::Zero()) {
//         pronto::quadruped::LegVectorMap res;
//         getFeetGRF(q, qd, tau, orient, res, qdd, xd, xdd, omega, omegad);
//         return res;
//     }

//     inline void setContactPoint(LegID leg, double foot_x, double foot_y){

//     }

private:
    JYPro::rcg::InertiaProperties inertia_prop_;
    JYPro::rcg::MotionTransforms motion_transf_;
    JYPro::rcg::ForceTransforms force_transf_;
    JYPro::rcg::InverseDynamics inverse_dynamics_;
    JYPro::rcg::JSIM jsim_;
    FeetJacobians feet_jacs_;

};

}
}
