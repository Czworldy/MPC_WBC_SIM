#ifndef CONTROLFSMDATA_H
#define CONTROLFSMDATA_H

#include "UserParameter.h"
#include "BodyStateEstimator.h"
#include "LegStateEstimator.h"

template <typename T>
struct ControlFSMData {
    BodyStateEstData<T> bodyStateEst;
    LegStateEstData<T> legStateEst[4];//与WBC的索引保持一致
};

template struct ControlFSMData<double>;
template struct ControlFSMData<float>;

#endif