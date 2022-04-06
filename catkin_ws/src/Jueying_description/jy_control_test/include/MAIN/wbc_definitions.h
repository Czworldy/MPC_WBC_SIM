#ifndef WBC_DEFINITION_H
#define WBC_DEFINITION_H

#include <cmath>
#include "cppTypes.h"

typedef struct
{
	Vec31<float> x;
	Vec31<float> y;
	Vec31<float> z;
	Vec31<float> roll;
	Vec31<float> pitch;
	Vec31<float> yaw;
}  BaseStatesForPlan;

typedef struct
{
	Vec31<float> x;
	Vec31<float> y;
	Vec31<float> z;
}  FootStatesForPlan;

struct LimbsContacts {
public: 
    float lf;
    float rf;
    float lh;
    float rh;
};

typedef struct
{
	double value[3];
} OneLimbData;

typedef struct
{
	OneLimbData lf_pos;
	OneLimbData rf_pos;
	OneLimbData lh_pos;
	OneLimbData rh_pos;
	OneLimbData lf_vel;
	OneLimbData rf_vel;
	OneLimbData lh_vel;
	OneLimbData rh_vel;
}	LimbsPosVel;

typedef struct
{
    OneLimbData lf_tau;
    OneLimbData rf_tau;
    OneLimbData lh_tau;
    OneLimbData rh_tau;
}   LimbsCommand;

struct EstimatorOutput {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    float time_stamp;
    Eigen::Matrix<double, 3, 1> base_pos_world;
    Eigen::Matrix<double, 3, 1> base_linear_vel_world;
    Eigen::Matrix<double, 3, 1> base_linear_vel_body;
    Eigen::Quaterniond base_orientation_world;
    Eigen::Matrix<double, 3, 1> base_angular_vel_world;
    Eigen::Matrix<double, 3, 1> base_angular_vel_body;
    LimbsContacts contact;
    LimbsPosVel jointStates;

    Eigen::Matrix<double, 3, 1> frame_c_rpy_in_world;
    Eigen::Quaterniond frame_c_quat_in_world;
    Eigen::Matrix<double, 3, 1> frame_c_xyz_in_world;
};

// for MPC comunication  float base
struct conversionData{
    vector_array2_t swingFeetPosition; //lf lh rf rh
    vector_array2_t swingFeetVelocity; //lf lh rf rh
    vector_array2_t swingFeetAcceleration;  //lf lh rf rh
    Eigen::Vector4f firstGait; //lf lh rf rh
    Eigen::Vector4f secondGait; //lf lh rf rh
    Eigen::Vector4f thirdGait; //lf lh rf rh
    Eigen::Vector2f switchTime;
    vector_array_t basePosition;
    vector_array_t baseVelocity;
    vector_array_t baseAcceleration;
    Eigen::VectorXf stateTime;
};

#endif