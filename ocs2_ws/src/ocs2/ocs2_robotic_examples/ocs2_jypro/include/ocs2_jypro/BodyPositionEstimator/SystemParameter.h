#ifndef SYSTEMPARAMETER_H
#define SYSTEMPARAMETER_H

#include "cppTypes.h"

namespace legID{
    constexpr size_t LF = 0;
    constexpr size_t LB = 1;
    constexpr size_t RF = 2;
    constexpr size_t RB = 3;
}


namespace legID_P{
    constexpr size_t LF = 0;
    constexpr size_t LB = 1;
    constexpr size_t RB = 2;
    constexpr size_t RF = 3;
}//counter-clockwise

template<typename T>
class SystemParameter{
    public:
        //Hip Location in the Body Frame
        Vec31<T> hipLocation[4];

        //Physical parameters of leg
        T deviation; 
        T thigh;
        T shank;

        //Foot Location in the Knee Frame
        Vec31<T> footLocation;
        Vec41<T> footLocation_q;

        //Foot Location in Body Frame
        Vec31<T> getFootLocation(const Vec31<T>& q, size_t legID);
        Vec31<T> getFootLocationLF(const Vec31<T>& q_LF);
        Vec31<T> getFootLocationLB(const Vec31<T>& q_LB);
        Vec31<T> getFootLocationRB(const Vec31<T>& q_RB);
        Vec31<T> getFootLocationRF(const Vec31<T>& q_RF);

        //Foot Jacobian in Body Frame
        Mat3<T> getFootJacobianLF(const Vec31<T>& q_LF);
        Mat3<T> getFootJacobianLB(const Vec31<T>& q_LB);
        Mat3<T> getFootJacobianRB(const Vec31<T>& q_RB);
        Mat3<T> getFootJacobianRF(const Vec31<T>& q_RF);


        SystemParameter();
        ~SystemParameter();
};
#endif