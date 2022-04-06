#ifndef UTILITY_H
#define UTILITY_H

#include<cppTypes.h>
#include <eigen3/Eigen/Geometry>
#include<cmath>

class quaternionToRad
{

public:
    int circle_counter = 0;
    float preYaw = 0;

    Vec31<float> quaternionToTotalRad(Eigen::Quaternion<float> q){
        Vec31<float> rpy;
        rpy[0] = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(pow(q.x(), 2)+pow(q.y(), 2)));
        rpy[1] = asin(2*(q.w()*q.y() - q.z()*q.x()));
        rpy[2] = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(pow(q.y(), 2)+pow(q.z(), 2)));

        float deltaYaw = rpy[2] - preYaw;
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

    quaternionToRad(){reset();}
};
// extern template class quaternionToRad<double>;
// extern template class quaternionToRad<float>;

// template<typename T>
// Vec31<T> quaternionToRad<T>::quaternionToTotalRad(Eigen::Quaternion<float> q){
//     Vec31<T> rpy;
//     rpy[0] = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(pow(q.x(), 2)+pow(q.y(), 2)));
//     rpy[1] = asin(2*(q.w()*q.y() - q.z()*q.x()));
//     rpy[2] = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(pow(q.y(), 2)+pow(q.z(), 2)));

//     T deltaYaw = rpy[2] - preYaw;
//     if(deltaYaw > 1.6*M_PI){
//         circle_counter--;
//     }
//     else if(deltaYaw < -1.6*M_PI){
//         circle_counter++;
//     }
//     preYaw = rpy[2];
//     rpy[2] = rpy[2] + 2*circle_counter*M_PI;

//     return rpy;
// }



template<typename T>
Vec31<T> quaternionTOrpy(Eigen::Quaternion<T> q){
    Vec31<T> rpy;
    rpy[0] = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(pow(q.x(), 2)+pow(q.y(), 2)));
    rpy[1] = asin(2*(q.w()*q.y() - q.z()*q.x()));
    rpy[2] = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(pow(q.y(), 2)+pow(q.z(), 2)));
    // if(rpy[2] < 0)
    //     rpy[2] += 2*M_PI;
    return rpy;
 };

template<typename T>
Eigen::Quaternion<T> rpyTOquaternion(Vec31<T> rpy){
    Eigen::Quaternion<T> q;
    const T roll = rpy[0], pitch = rpy[1], yaw = rpy[2];
    q.w() = cos(pitch / 2.0f) * cos(roll / 2.0f) * cos(yaw / 2.0f) - sin(pitch / 2.0f) * sin(roll / 2.0f) * sin(yaw / 2.0f);
    q.x() = cos(roll / 2.0f) * cos(yaw / 2.0f) * sin(pitch / 2.0f) - cos(pitch / 2.0f) * sin(roll / 2.0f) * sin(yaw / 2.0f);
    q.y() = cos(pitch / 2) * cos(yaw / 2.0f) * sin(roll /2.0f) + cos(roll / 2.0f) * sin(pitch / 2.0f) * sin(yaw / 2.0f);
    q.z() = cos(pitch / 2.0f) * cos(roll / 2.0f) * sin(yaw / 2.0f) + cos(yaw /2.0f) * sin(pitch / 2.0f) * sin(roll / 2.0f);

    return q;
}

template<typename T>
Mat3<T> rpyTORotateMat(T roll, T pitch, T yaw){
    Mat3<T> RotateMatrix, R_roll, R_pitch, R_yaw;
    R_roll <<  1., 0., 0., 
               0., cos(roll), -sin(roll),
               0., sin(roll), cos(roll);
    R_pitch << cos(pitch), 0, sin(pitch),
               0., 1., 0.,
               -sin(pitch), 0., cos(pitch);
    R_yaw << cos(yaw), -sin(yaw), 0.,
             sin(yaw), cos(yaw), 0.,
             0., 0., 1.;
    RotateMatrix = R_yaw * R_pitch * R_roll;
    return RotateMatrix;
}

template<typename T>
Eigen::Quaternion<T> rpyTOquaternion(T roll, T pitch, T yaw){
    Eigen::Quaternion<T> q(rpyTORotateMat(roll,pitch,yaw));
    return q;
}


template<typename T>
Mat3<T> changeFrame(T yaw){
    Mat3<T> R_yaw;
    R_yaw << cos(yaw), sin(yaw), 0.,
             -sin(yaw),cos(yaw), 0.,
             0., 0., 1.;
    return R_yaw;
}

template<typename T>
Eigen::Matrix<T, 3, 3> rpyDotTOtwist(T roll, T pitch, T yaw){
        Eigen::Matrix<T, 3, 3> translation_Matrix;
    translation_Matrix << cos(pitch) * cos(yaw), -sin(yaw), 0,
                          cos(pitch) * sin(yaw), cos(yaw), 0,
                          -sin(pitch), 0 , 1;

    return translation_Matrix;
}


template <typename T>
void quaternionToso3(const Eigen::Quaternion<T> quat, Vec31<T>& so3) {
  so3[0] = quat.x();
  so3[1] = quat.y();
  so3[2] = quat.z();

  T theta =
      2.0 * asin(sqrt(so3[0] * so3[0] + so3[1] * so3[1] + so3[2] * so3[2]));

  if (fabs(theta) < 0.0000001) {
    so3.setZero();
    return;
  }
  so3 /= sin(theta / 2.0);
  so3 *= theta;
}


#endif