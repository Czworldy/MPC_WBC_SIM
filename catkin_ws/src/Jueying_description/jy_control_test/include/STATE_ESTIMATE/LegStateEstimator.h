#ifndef LEGSTATEESTIMATOR_H
#define LEGSTATEESTIMATOR_H

#include "cppTypes.h"

//TODO: Conmmunicate with ROS
template<typename T>
struct LegStateEstData{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec31<T> q;
    Vec31<T> qd;
};

template struct LegStateEstData<double>;
template struct LegStateEstData<float>;

#endif