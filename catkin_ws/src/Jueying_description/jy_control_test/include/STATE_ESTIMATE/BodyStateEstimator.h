#ifndef BODYSTATEESTIMATOR_H
#define BODYSTATEESTIMATOR_H

#include "cppTypes.h"

//TODO: Conmmunicate with ROS
template<typename T>
struct BodyStateEstData{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vec41<float> contactEstimate;//与WBC的索引保持一致
    Vec41<float> contactEstimate_P;//与轨迹规划的索引保持一致
    Vec31<T> base_pos_world;
    Vec31<T> base_linear_vel_world;
    Vec31<T> base_linear_vel_body;
    Eigen::Quaternion<T> base_orientation_world;
    Vec31<T> base_angular_vel_world;
    Vec31<T> base_angular_vel_body;

    Vec31<T> base_rpy_world;
    Mat3<T> base_rotMat_world;

    Vec31<T> frame_c_rpy_in_world;
    Eigen::Quaternion<T> frame_c_quat_in_world;
    Vec31<T> frame_c_xyz_in_world;
};

template struct BodyStateEstData<double>;
template struct BodyStateEstData<float>;

#endif

