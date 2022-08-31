#ifndef QUADRUPED_DYNAMICS_MODEL_H
#define QUADRUPED_DYNAMICS_MODEL_H

//ROS related
#include<ros/ros.h>
#include<ros/node_handle.h>

//RBDL related
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include <rbdl/Model.h>
#include <rbdl/Dynamics.h>
#include <rbdl/Kinematics.h>
#include <rbdl/Constraints.h>

#include<cppTypes.h>
#include"UserParameter.h"
#include<eigen3/Eigen/Geometry>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

//The State of a Quadruped robot_______________DATA_______________
template <typename T>
struct FBModelState{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Quaternion<T> bodyOrientation;
    Vec31<T> bodyRPY;
    Vec31<T> bodyPosition;
    SVec<T> bodyVelocity;
    Vec12<T> q_leg;
    Vec12<T> qd_leg;

    Vec31<T> frame_c_rpy_in_world;
    Eigen::Quaternion<T> frame_c_quat_in_world;
    Vec31<T> frame_c_xyz_in_world;

    Vec41<float> contact_state_;
};

namespace JYPro{
    constexpr size_t num_act_joint = 12;
    constexpr size_t dim_config = 18;
    constexpr size_t num_leg = 4;
    constexpr size_t num_leg_joint = 3;    
}

namespace legID{
    constexpr size_t LF = 0;
    constexpr size_t LB = 1;
    constexpr size_t RF = 2;
    constexpr size_t RB = 3;
}


class QuadrupedDynamicsModel
{
public:
    QuadrupedDynamicsModel();
    ~QuadrupedDynamicsModel(){delete quadmodel;}


    void setState(const FBModelState<double>& state);
    void massMatrix();
    void nonlinearEffect();
    void contactJacobian();
    void CoM6DJacobian();
    void CoM6DJacobian_c_frame();
    void transMatForTrackingTasks();
    const DMat<double>& swingFootJacobian(size_t foot_id);
    const DMat<double>& swingFootJacobian_c_frame(size_t foot_id);
    DVec<double> swingFootPosition(size_t foot_id);
    DVec<double> swingFootPosition_c_frame(size_t foot_id);
    DVec<float> swingFootPosition(size_t foot_id, const VectorNd &given_Q);
    DVec<double> swingFootVelocity(size_t foot_id);
    DVec<double> swingFootVelocity_c_frame(size_t foot_id);


    Vec31<double> hipPosition(size_t hip_id);//for motion plan
    Vec31<double> hipVelocity(size_t hip_id);//for motion plan
    Vec31<double> footPosition(size_t foot_id);//for motion plan

    const DMat<double> & getMassMatrix()  {return _H;}
    const DVec<double> & getNolinearEffect()  {return _N;}
    const DMat<double>&  getContactJacobian()  {return _CJ;}
    DMat<double> getCoM6DJacobian();
    DMat<double> getCoM6DJacobian_c_frame();
    const DVec<double> & getCoM6DJDotQDot();
    const DVec<double> & getCoM6DJDotQDot_c_frame();
    const DVec<double>& getCJDotQDot();
    
    const Vec31<double>& get_CoM_Position();
    const Vec31<double>& get_CoM_in_BaseFrame(const Vector3d& CoM_Pos);
    const Vec31<double>& get_CoM_Velocity();
    Vec31<double> get_Base_Velocity_from_CoM(const Vector3d& CoM_vel);
    Vec31<double> get_Base_Position_from_CoM(const Vector3d& CoM_Pos);
 
    // VectorNd Q;
    Eigen::Matrix<double,19,1> Q;
    Eigen::Matrix<double,19,1> Q_c_frame;
    Vector3d angularTwist;
    Eigen::Matrix<double,18,1> QDot;
    Eigen::Matrix<double,18,1> QDot_c_frame;
    size_t num_contact;
    double duration;
    Vec41<float> contact_state;
    Matrix3d rotMat_world_to_c;
    Eigen::Quaterniond quat_world_to_c;
    Vec31<double> xyz_c_to_world;

    MatrixNd rotMatForTracking;

    
protected:

    void initParameters();
    void generate();

    Model * quadmodel;

    enum BodyName{
        Base,
        LF_Hip,
        LF_Thigh,
        LF_Shank,
        LB_Hip,
        LB_Thigh,
        LB_Shank,
        RF_Hip,
        RF_Thigh,
        RF_Shank,
        RB_Hip,
        RB_Thigh,
        RB_Shank,
        BodyNameCount
    };

    enum JointName{
        FloatBase,
        LF_HipX,
        LF_HipY,
        LF_Knee,
        LB_HipX,
        LB_HipY,
        LB_Knee,
        RF_HipX,
        RF_HipY,
        RF_Knee,
        RB_HipX,
        RB_HipY,
        RB_Knee,
        JointNameCount
    };

    struct Transfrom
    {
        Matrix3d R;
        Vector3d Tr;
    };
    
    double BodyMass[BodyNameCount];
    Vector3d BodyCoM[BodyNameCount];
    Matrix3d BodyInertia[BodyNameCount];
    Transfrom Trans[JointNameCount];
    unsigned int body_id[BodyNameCount];
    Vector3d contact_point;
    Vector3d com_position;
    Vector3d com_velocity;
    Vector3d com_in_base;
    Vector3d hip_position;
    Vector3d hip_velocity;

    MatrixNd  _H;
    VectorNd  _N;
    MatrixNd  _CJ,_FootJ, G;
    MatrixNd  _CJ_pre, _FootJ_pre;
    VectorNd  _CJDotQDot;
    MatrixNd _JCoM, Jcom;
    MatrixNd _Jcom_pre;
    VectorNd _JCoMDotQDot;
    MatrixNd _JSwingFoot;

    MatrixNd _JCoM_c_frame, Jcom_c_frame;
    MatrixNd _Jcom_pre_c_frame;
    VectorNd _JCoMDotQDot_c_frame;
    MatrixNd _JSwingFoot_c_frame;

    ConstraintSet constraint_set;
    Vector3d _foot_position;
    Vector3d _foot_velocity;
    UserParameter<double> paramd;
};

#endif

