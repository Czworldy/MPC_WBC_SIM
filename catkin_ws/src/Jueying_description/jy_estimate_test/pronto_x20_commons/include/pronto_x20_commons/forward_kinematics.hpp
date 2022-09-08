#pragma once

#include <pronto_quadruped_commons/forward_kinematics.h>
#include <x20_robcogen/transforms.h>

namespace X20 {
namespace rcg {

class ForwardKinematics : public pronto::quadruped::ForwardKinematics {
public:
    typedef pronto::quadruped::JointState JointState;
    typedef pronto::quadruped::LegID LegID;
    typedef pronto::quadruped::Vector3d Vector3d;
    typedef pronto::quadruped::Matrix3d Matrix3d;

public:
    ForwardKinematics();

    ~ForwardKinematics(){}

    Vector3d getFootPosLF(const JointState& q);

    Vector3d getFootPosRF(const JointState& q);

    Vector3d getFootPosLH(const JointState& q);

    Vector3d getFootPosRH(const JointState& q);

    Vector3d getFootPos(const JointState& q, const LegID& leg);

    Matrix3d getFootOrientation(const JointState &q, const LegID &leg);

    Vector3d getShinPos(const JointState& q,
                        const double& contact_pos,
                        const LegID& leg);

private:
    X20::rcg::HomogeneousTransforms ht_;
};

}
}