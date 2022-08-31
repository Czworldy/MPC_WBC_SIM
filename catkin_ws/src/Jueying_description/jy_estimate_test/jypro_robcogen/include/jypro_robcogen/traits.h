#ifndef RCG__JYPRO_TRAITS_H_
#define RCG__JYPRO_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace JYPro {
namespace rcg {
struct Traits {
    typedef typename JYPro::rcg::ScalarTraits ScalarTraits;

    typedef typename JYPro::rcg::JointState JointState;

    typedef typename JYPro::rcg::JointIdentifiers JointID;
    typedef typename JYPro::rcg::LinkIdentifiers  LinkID;

    typedef typename JYPro::rcg::HomogeneousTransforms HomogeneousTransforms;
    typedef typename JYPro::rcg::MotionTransforms MotionTransforms;
    typedef typename JYPro::rcg::ForceTransforms ForceTransforms;

    typedef typename JYPro::rcg::InertiaProperties InertiaProperties;
    typedef typename JYPro::rcg::ForwardDynamics FwdDynEngine;
    typedef typename JYPro::rcg::InverseDynamics InvDynEngine;
    typedef typename JYPro::rcg::JSIM JSIM;

    static const int joints_count = JYPro::rcg::jointsCount;
    static const int links_count  = JYPro::rcg::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return JYPro::rcg::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return JYPro::rcg::orderedLinkIDs;
}

}
}

#endif
