#ifndef MOTIONPLANDATA_H
#define MOTIONPLANDATA_H

#include "cppTypes.h"

struct DesMotionData{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Matrix<double, 24,1> splineX; 
    Eigen::Matrix<double, 24,1> splineY;
    Vec31<float> swingFoot_p_[4];
    Vec31<float> swingFoot_v_[4];
    Vec31<float> swingFoot_a_[4];
    Vec41<float> contactStateIter;
    Vec2<float> footLocation_lf;
    Vec2<float> footLocation_lb;
    Vec2<float> footLocation_rf;
    Vec2<float> footLocation_rb;
    long long int iterCounter;
    bool if_solved;
};

#endif