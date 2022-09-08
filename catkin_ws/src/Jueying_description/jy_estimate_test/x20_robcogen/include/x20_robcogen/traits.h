#ifndef RCG__X20_TRAITS_H_
#define RCG__X20_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"

namespace X20 {
namespace rcg {
struct Traits {
    typedef typename X20::rcg::ScalarTraits ScalarTraits;

    typedef typename X20::rcg::JointState JointState;

    typedef typename X20::rcg::JointIdentifiers JointID;
    typedef typename X20::rcg::LinkIdentifiers  LinkID;

    typedef typename X20::rcg::HomogeneousTransforms HomogeneousTransforms;
    typedef typename X20::rcg::MotionTransforms MotionTransforms;
    typedef typename X20::rcg::ForceTransforms ForceTransforms;

    typedef typename X20::rcg::InertiaProperties InertiaProperties;
    typedef typename X20::rcg::ForwardDynamics FwdDynEngine;
    typedef typename X20::rcg::InverseDynamics InvDynEngine;
    typedef typename X20::rcg::JSIM JSIM;

    static const int joints_count = X20::rcg::jointsCount;
    static const int links_count  = X20::rcg::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};


inline const Traits::JointID*  Traits::orderedJointIDs() {
    return X20::rcg::orderedJointIDs;
}
inline const Traits::LinkID*  Traits::orderedLinkIDs() {
    return X20::rcg::orderedLinkIDs;
}

}
}

#endif
