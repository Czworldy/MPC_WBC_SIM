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
    void clear() {
        value[0] = 0.0;
        value[1] = 0.0;
        value[2] = 0.0;
    }
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
    void clear(){
        lf_pos.clear();
        rf_pos.clear();
        lh_pos.clear();
        rh_pos.clear();
        lf_vel.clear();
        rf_vel.clear();
        lh_vel.clear();
        rh_vel.clear();
    }
}	LimbsPosVel;

typedef struct
{
    OneLimbData lf_tau;
    OneLimbData rf_tau;
    OneLimbData lh_tau;
    OneLimbData rh_tau;
}   LimbsCommand;

struct TerrainEstData{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
public:
    Eigen::Quaternionf terrainQuat;
    Eigen::Vector3f terrainParams;
    Eigen::Vector4f feetHeight; // lf lh rf rh
};

struct EstimatorOutput {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    EstimatorOutput(){ clear(); }
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

    TerrainEstData terrainEstData;

    void clear(){
        time_stamp = 0;
        base_pos_world.setZero();
        base_linear_vel_world.setZero();
        base_linear_vel_body.setZero();
        base_orientation_world.setIdentity();
        base_angular_vel_world.setZero();
        base_angular_vel_body.setZero();
        contact.lf = 0;
        contact.rf = 0;
        contact.lh = 0;
        contact.rh = 0;
        jointStates.clear();
        frame_c_rpy_in_world.setZero();
        frame_c_quat_in_world.setIdentity();
        frame_c_xyz_in_world.setZero();
        terrainEstData.terrainQuat.setIdentity();
        terrainEstData.terrainParams.setZero();
    }
};

// for MPC comunication  float base
struct conversionData{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
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

    vector_array_t actJointPos;
    vector_array_t actJointVel;
};

// for MPC comunication  float base
// #define LENGTH 15
// using vector_foot_t = Eigen::Matrix <Eigen::Matrix<Eigen::Matrix<float, 3, 1 >, 4, 1> ,LENGTH ,1>;
// using vector_base_t = Eigen::Matrix <Eigen::Matrix<float, 6, 1> ,LENGTH ,1>;
// using vector_joint_t = Eigen::Matrix <Eigen::Matrix<float, 12, 1> ,LENGTH ,1>;
// struct conversionData{
// public:
//         EIGEN_MAKE_ALIGNED_OPERATOR_NEW
// public:
//     vector_foot_t swingFeetPosition; //lf lh rf rh
//     vector_foot_t swingFeetVelocity; //lf lh rf rh
//     vector_foot_t swingFeetAcceleration;  //lf lh rf rh
//     Eigen::Vector4f firstGait; //lf lh rf rh
//     Eigen::Vector4f secondGait; //lf lh rf rh
//     Eigen::Vector4f thirdGait; //lf lh rf rh
//     Eigen::Vector2f switchTime;
//     vector_base_t basePosition;
//     vector_base_t baseVelocity;
//     vector_base_t baseAcceleration;
//     vector_joint_t actJointPos;
//     vector_joint_t actJointVel;
//     Eigen::Matrix<float, LENGTH ,1> stateTime;
// };

#endif