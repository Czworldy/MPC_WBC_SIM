#pragma once

#include "cppTypes.h"
#include "ocs2_core/Types.h"
namespace ocs2 {

class QuaternionToRPY
{

public:
    int circle_counter = 0;
    scalar_t preYaw = 0;

    Eigen::Matrix<scalar_t, 3, 1> quaternionToTotalRad(const Eigen::Quaternion<scalar_t>& q){
        Eigen::Matrix<scalar_t, 3, 1> rpy;
        rpy[0] = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(pow(q.x(), 2)+pow(q.y(), 2)));
        rpy[1] = asin(2*(q.w()*q.y() - q.z()*q.x()));
        rpy[2] = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(pow(q.y(), 2)+pow(q.z(), 2)));

        scalar_t deltaYaw = rpy[2] - preYaw;
        if(deltaYaw > 1.6*M_PI){
            circle_counter--;
        }
        else if(deltaYaw < -1.6*M_PI){
            circle_counter++;
        }
        preYaw = rpy[2];
        rpy[2] = rpy[2] + 2*circle_counter*M_PI;

        return rpy;
    }
    
    void reset(){circle_counter = 0; preYaw = 0;}

    QuaternionToRPY(){reset();}
};
} //namespace ocs2

