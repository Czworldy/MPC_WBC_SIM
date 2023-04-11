#pragma GCC optimize(2)
#include <WBC_CONTROL/dynamics/quadruped_dynamics_model.h>
#include<Math/utility.h>
#include<iostream>
#include"Math/utility.h"

QuadrupedDynamicsModel::QuadrupedDynamicsModel(){
    rbdl_check_api_version (RBDL_API_VERSION);
    duration = paramd.cycle_time;
    quadmodel = new Model();
    initParameters();
    generate();

    // ROS_INFO_STREAM("Degree of freedom overview: "<<Utils::GetModelDOFOverview(*quadmodel));
    // ROS_INFO_STREAM("Model Hierarchy: "<<Utils::GetModelHierarchy(*quadmodel));
    // ROS_INFO("Finished QuadrupedDynamicsModel init!");
    // ROS_INFO_STREAM("Q_SIZE "<< quadmodel->q_size);
    // ROS_INFO_STREAM("QDOT_SIZE "<< quadmodel->qdot_size);
    // ROS_INFO_STREAM("BODYID "<< body_id[0]);
    // ROS_INFO_STREAM("BODYID "<< body_id[1]);
    // ROS_INFO_STREAM("BODYID "<< body_id[2]);
    // ROS_INFO_STREAM("BODYID "<< body_id[3]);
    // ROS_INFO_STREAM("BODYID "<< body_id[4]);
    // ROS_INFO_STREAM("BODYID "<< body_id[5]);
    // ROS_INFO_STREAM("BODYID "<< body_id[6]);
    // ROS_INFO_STREAM("BODYID "<< body_id[7]);
    // ROS_INFO_STREAM("BODYID "<< body_id[8]);
    // ROS_INFO_STREAM("BODYID "<< body_id[9]);
    // ROS_INFO_STREAM("BODYID "<< body_id[10]);
    // ROS_INFO_STREAM("BODYID "<< body_id[11]);
    // ROS_INFO_STREAM("BODYID "<< body_id[12]);

    Q = VectorNd::Zero(quadmodel->q_size);
    QDot = VectorNd::Zero(quadmodel->qdot_size);
    _H = MatrixNd::Zero(quadmodel->qdot_size, quadmodel->qdot_size);
    _N = VectorNd::Zero(quadmodel->qdot_size);
    _JCoM = MatrixNd::Zero(6, quadmodel->qdot_size);
    Jcom = MatrixNd::Zero(6, quadmodel->qdot_size);
    _Jcom_pre = MatrixNd::Zero(6, quadmodel->qdot_size);
    _JCoMDotQDot = VectorNd::Zero(6);
    _JSwingFoot = MatrixNd::Zero(3, quadmodel->qdot_size);
    G = MatrixNd::Zero(3, quadmodel->qdot_size);

    _FootJ = MatrixNd::Zero(12, quadmodel->qdot_size);
    _FootJ_pre = MatrixNd::Zero(12, quadmodel->qdot_size);

    Q_c_frame = VectorNd::Zero(quadmodel->q_size);
    QDot_c_frame = VectorNd::Zero(quadmodel->qdot_size);
    _JCoM_c_frame = MatrixNd::Zero(6, quadmodel->qdot_size);
    Jcom_c_frame = MatrixNd::Zero(6, quadmodel->qdot_size);
    _Jcom_pre_c_frame = MatrixNd::Zero(6, quadmodel->qdot_size);
    _JCoMDotQDot_c_frame = VectorNd::Zero(6);
    _JSwingFoot_c_frame = MatrixNd::Zero(3, quadmodel->qdot_size);

}
//   0: Base_TX
//   1: Base_TY
//   2: Base_TZ
//   3: Base_RX
//   4: Base_RY
//   5: Base_RZ
//   6: LF_Hip_TX
//   7: LF_Thigh_custom_axis ( 0  0  0  0  0 -1)
//   8: LF_Shank_custom_axis ( 0  0  0  0  0 -1)
//   9: LB_Hip_TX
//  10: LB_Thigh_custom_axis ( 0  0  0  0  0 -1)
//  11: LB_Shank_custom_axis ( 0  0  0  0  0 -1)
//  12: RF_Hip_custom_axis ( 0  0  0 -1  0  0)
//  13: RF_Thigh_custom_axis ( 0  0  0  0  0 -1)
//  14: RF_Shank_custom_axis ( 0  0  0  0  0 -1)
//  15: RB_Hip_custom_axis ( 0  0  0 -1  0  0)
//  16: RB_Thigh_custom_axis ( 0  0  0  0  0 -1)
//  17: RB_Shank_custom_axis ( 0  0  0  0  0 -1)


void QuadrupedDynamicsModel::setState(const FBModelState<double>& state){

    // Q.head(3) = state.bodyPosition;
    // Q[3]  = state.bodyOrientation.x();
    // Q[4]  = state.bodyOrientation.y();
    // Q[5]  = state.bodyOrientation.z();
    // Q[18] = state.bodyOrientation.w();
    // Q.segment(6, JYPro::num_act_joint) = state.q_leg;

    // QDot.head(3) = state.bodyVelocity.head(3);
    // QDot.segment(3,3) = state.bodyVelocity.segment(3,3); 
    // QDot.tail(JYPro::num_act_joint) = state.qd_leg; 

    rotMat_world_to_c = state.frame_c_quat_in_world.toRotationMatrix().transpose();
    quat_world_to_c = rotMat_world_to_c;
    xyz_c_to_world = state.frame_c_xyz_in_world;

    Q.head(3) = state.bodyPosition;
    Eigen::Quaterniond quat_base_in_c(quat_world_to_c * state.bodyOrientation);
    // Q[3]  = quat_base_in_c.x();
    // Q[4]  = quat_base_in_c.y();
    // Q[5]  = quat_base_in_c.z();
    // Q[18] = quat_base_in_c.w();
    Q[3]  = state.bodyOrientation.x();
    Q[4]  = state.bodyOrientation.y();
    Q[5]  = state.bodyOrientation.z();
    Q[18] = state.bodyOrientation.w();
    Q.segment(6, JYPro::num_act_joint) = state.q_leg;

    // QDot.head(3) = rotMat_world_to_c * state.bodyVelocity.head(3); 
    QDot.head(3) = state.bodyVelocity.head(3);//yjy :按照论文 线速度先不旋转
    QDot.segment(3,3) = state.bodyVelocity.segment(3,3); 
    QDot.tail(JYPro::num_act_joint) = state.qd_leg; 

    Q_c_frame.head(3) = rotMat_world_to_c * (state.bodyPosition - xyz_c_to_world); // to check
    quat_base_in_c = quat_world_to_c * state.bodyOrientation;
    Q_c_frame[3]  = quat_base_in_c.x();
    Q_c_frame[4]  = quat_base_in_c.y();
    Q_c_frame[5]  = quat_base_in_c.z();
    Q_c_frame[18] = quat_base_in_c.w();
    // Q_c_frame[3]  = state.bodyOrientation.x();
    // Q_c_frame[4]  = state.bodyOrientation.y();
    // Q_c_frame[5]  = state.bodyOrientation.z();
    // Q_c_frame[18] = state.bodyOrientation.w();
    Q_c_frame.segment(6, JYPro::num_act_joint) = state.q_leg;

    QDot_c_frame.head(3) = rotMat_world_to_c * state.bodyVelocity.head(3);
    // std::cout << "QDot_c_frame.head(3) = " << QDot_c_frame.head(3).transpose() << std::endl;
    QDot_c_frame.segment(3,3) = rotMat_world_to_c * state.bodyVelocity.segment(3,3); 
    QDot_c_frame.tail(JYPro::num_act_joint) = state.qd_leg; 

    contact_state = state.contact_state_;

    //contact related
    num_contact = 0;
    constraint_set.clear();
    if(state.contact_state_[legID::LF]){
        num_contact++;
    }
    if(state.contact_state_[legID::LB]){
        num_contact++;
    }
    if(state.contact_state_[legID::RF]){
        num_contact++;
    }
    if(state.contact_state_[legID::RB]){
        num_contact++;
    }

    _CJ = MatrixNd::Zero(3*num_contact, quadmodel->qdot_size);
    _CJ_pre= MatrixNd::Zero(3*num_contact, quadmodel->qdot_size);
    _CJDotQDot =  VectorNd::Zero(3*num_contact);

    rotMatForTracking = MatrixNd::Identity(quadmodel->qdot_size + 3*num_contact, quadmodel->qdot_size + 3*num_contact);
}


void QuadrupedDynamicsModel::massMatrix(){
    _H.setZero();
    CompositeRigidBodyAlgorithm(*quadmodel, Q, _H, true);
}


void QuadrupedDynamicsModel::nonlinearEffect(){
    NonlinearEffects(*quadmodel, Q, QDot, _N);
}


void QuadrupedDynamicsModel::contactJacobian(){
    _FootJ_pre = _FootJ;
    int leg = 0;
    //CalcConstraintsJacobian(*quadmodel, Q, constraint_set, _CJ);
    G.setZero();
    CalcPointJacobian(*quadmodel, Q, body_id[LF_Shank], contact_point, G);
    _FootJ.block(0,0,3,quadmodel->qdot_size) = G;
    if(contact_state[legID::LF]){
        _CJ.block(leg, 0, 3, quadmodel->qdot_size) = G;
        _CJDotQDot.segment(leg,3) = (G - _FootJ_pre.block(0, 0, 3, quadmodel->qdot_size))/duration*QDot;
        leg+=3;
    }
    G.setZero();
    CalcPointJacobian(*quadmodel, Q, body_id[LB_Shank], contact_point, G);
    _FootJ.block(3,0,3,quadmodel->qdot_size) = G;
    if(contact_state[legID::LB]){
        _CJ.block(leg, 0, 3, quadmodel->qdot_size) = G;
        _CJDotQDot.segment(leg,3) = (G - _FootJ_pre.block(3, 0, 3, quadmodel->qdot_size))/duration*QDot;
        leg+=3;
    }
    G.setZero();
    CalcPointJacobian(*quadmodel, Q, body_id[RF_Shank], contact_point, G);
    _FootJ.block(6,0,3,quadmodel->qdot_size) = G;
    if(contact_state[legID::RF]){
        _CJ.block(leg, 0, 3, quadmodel->qdot_size) = G;
        _CJDotQDot.segment(leg,3) = (G - _FootJ_pre.block(6, 0, 3, quadmodel->qdot_size))/duration*QDot;
        leg+=3;
    }
    G.setZero();
    CalcPointJacobian(*quadmodel, Q, body_id[RB_Shank], contact_point, G);
    _FootJ.block(9,0,3,quadmodel->qdot_size) = G;
    if(contact_state[legID::RB]){
        _CJ.block(leg, 0, 3, quadmodel->qdot_size) = G;
        _CJDotQDot.segment(leg,3) = (G - _FootJ_pre.block(9, 0, 3, quadmodel->qdot_size))/duration*QDot;
    }
}

void QuadrupedDynamicsModel::transMatForTrackingTasks() {
    // rotMatForTracking.block(0, 0, 3, 3) = rotMat_world_to_c;
    // rotMatForTracking.block(3, 3, 3, 3) = rotMat_world_to_c;

    // for (int i(0); i < num_contact; i++) {
    //     rotMatForTracking.block(quadmodel->qdot_size + 3*i, quadmodel->qdot_size + 3*i, 3, 3) = rotMat_world_to_c;
    // }
}

const Vec31<double>& QuadrupedDynamicsModel::get_CoM_Position(){
    Vector3d base_pos;
    Vector3d lf_hip_pos, lf_thigh_pos, lf_shank_pos;
    Vector3d lb_hip_pos, lb_thigh_pos, lb_shank_pos;
    Vector3d rf_hip_pos, rf_thigh_pos, rf_shank_pos;
    Vector3d rb_hip_pos, rb_thigh_pos, rb_shank_pos;

    base_pos     = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[Base],     BodyCoM[Base]);
    lf_hip_pos   = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LF_Hip],   BodyCoM[LF_Hip]);
    lf_thigh_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LF_Thigh], BodyCoM[LF_Thigh]);
    lf_shank_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LF_Shank], BodyCoM[LF_Shank]);
    lb_hip_pos   = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LB_Hip],   BodyCoM[LB_Hip]);
    lb_thigh_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LB_Thigh], BodyCoM[LB_Thigh]);
    lb_shank_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LB_Shank], BodyCoM[LB_Shank]);
    rf_hip_pos   = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RF_Hip],   BodyCoM[RF_Hip]);
    rf_thigh_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RF_Thigh], BodyCoM[RF_Thigh]);
    rf_shank_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RF_Shank], BodyCoM[RF_Shank]);
    rb_hip_pos   = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RB_Hip],   BodyCoM[RB_Hip]);
    rb_thigh_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RB_Thigh], BodyCoM[RB_Thigh]);
    rb_shank_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RB_Shank], BodyCoM[RB_Shank]);

    com_position = (BodyMass[Base]     * base_pos
                  + BodyMass[LF_Hip]   * lf_hip_pos
                  + BodyMass[LF_Thigh] * lf_thigh_pos
                  + BodyMass[LF_Shank] * lf_shank_pos
                  + BodyMass[LB_Hip]   * lb_hip_pos
                  + BodyMass[LB_Thigh] * lb_thigh_pos
                  + BodyMass[LB_Shank] * lb_shank_pos
                  + BodyMass[RF_Hip]   * rf_hip_pos
                  + BodyMass[RF_Thigh] * rf_thigh_pos
                  + BodyMass[RF_Shank] * rf_shank_pos
                  + BodyMass[RB_Hip]   * rb_hip_pos
                  + BodyMass[RB_Thigh] * rb_thigh_pos
                  + BodyMass[RB_Shank] * rb_shank_pos)/(BodyMass[Base] 
                  
                  + BodyMass[LF_Hip] + BodyMass[LF_Thigh] + BodyMass[LF_Shank]
                  + BodyMass[LB_Hip] + BodyMass[LB_Thigh] + BodyMass[LB_Shank]
                  + BodyMass[RF_Hip] + BodyMass[RF_Thigh] + BodyMass[RF_Shank]
                  + BodyMass[RB_Hip] + BodyMass[RB_Thigh] + BodyMass[RB_Shank]);
    
    return com_position;
}

Vec31<double> QuadrupedDynamicsModel::get_Base_Position_from_CoM(const Vector3d& CoM_Pos){
    Vector3d base_location;

    Vector3d lf_hip_pos, lf_thigh_pos, lf_shank_pos;
    Vector3d lb_hip_pos, lb_thigh_pos, lb_shank_pos;
    Vector3d rf_hip_pos, rf_thigh_pos, rf_shank_pos;
    Vector3d rb_hip_pos, rb_thigh_pos, rb_shank_pos;

    lf_hip_pos   = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LF_Hip],   BodyCoM[LF_Hip]);
    lf_thigh_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LF_Thigh], BodyCoM[LF_Thigh]);
    lf_shank_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LF_Shank], BodyCoM[LF_Shank]);
    lb_hip_pos   = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LB_Hip],   BodyCoM[LB_Hip]);
    lb_thigh_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LB_Thigh], BodyCoM[LB_Thigh]);
    lb_shank_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LB_Shank], BodyCoM[LB_Shank]);
    rf_hip_pos   = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RF_Hip],   BodyCoM[RF_Hip]);
    rf_thigh_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RF_Thigh], BodyCoM[RF_Thigh]);
    rf_shank_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RF_Shank], BodyCoM[RF_Shank]);
    rb_hip_pos   = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RB_Hip],   BodyCoM[RB_Hip]);
    rb_thigh_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RB_Thigh], BodyCoM[RB_Thigh]);
    rb_shank_pos = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RB_Shank], BodyCoM[RB_Shank]);

    base_location = ((BodyMass[Base] 
                  + BodyMass[LF_Hip] + BodyMass[LF_Thigh] + BodyMass[LF_Shank]
                  + BodyMass[LB_Hip] + BodyMass[LB_Thigh] + BodyMass[LB_Shank]
                  + BodyMass[RF_Hip] + BodyMass[RF_Thigh] + BodyMass[RF_Shank]
                  + BodyMass[RB_Hip] + BodyMass[RB_Thigh] + BodyMass[RB_Shank]) * CoM_Pos

                  -(BodyMass[LF_Hip]     * lf_hip_pos
                    + BodyMass[LF_Thigh] * lf_thigh_pos
                    + BodyMass[LF_Shank] * lf_shank_pos
                    + BodyMass[LB_Hip]   * lb_hip_pos
                    + BodyMass[LB_Thigh] * lb_thigh_pos
                    + BodyMass[LB_Shank] * lb_shank_pos
                    + BodyMass[RF_Hip]   * rf_hip_pos
                    + BodyMass[RF_Thigh] * rf_thigh_pos
                    + BodyMass[RF_Shank] * rf_shank_pos
                    + BodyMass[RB_Hip]   * rb_hip_pos
                    + BodyMass[RB_Thigh] * rb_thigh_pos
                    + BodyMass[RB_Shank] * rb_shank_pos))/BodyMass[Base];

    return base_location;
}

const Vec31<double>& QuadrupedDynamicsModel::get_CoM_Velocity(){
    Vector3d base_vel;
    Vector3d lf_hip_vel, lf_thigh_vel, lf_shank_vel;
    Vector3d lb_hip_vel, lb_thigh_vel, lb_shank_vel;
    Vector3d rf_hip_vel, rf_thigh_vel, rf_shank_vel;
    Vector3d rb_hip_vel, rb_thigh_vel, rb_shank_vel;

    base_vel     = CalcPointVelocity(*quadmodel, Q, QDot, body_id[Base],     BodyCoM[Base]);
    lf_hip_vel   = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LF_Hip],   BodyCoM[LF_Hip]);
    lf_thigh_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LF_Thigh], BodyCoM[LF_Thigh]);
    lf_shank_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LF_Shank], BodyCoM[LF_Shank]);
    lb_hip_vel   = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LB_Hip],   BodyCoM[LB_Hip]);
    lb_thigh_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LB_Thigh], BodyCoM[LB_Thigh]);
    lb_shank_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LB_Shank], BodyCoM[LB_Shank]);
    rf_hip_vel   = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RF_Hip],   BodyCoM[RF_Hip]);
    rf_thigh_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RF_Thigh], BodyCoM[RF_Thigh]);
    rf_shank_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RF_Shank], BodyCoM[RF_Shank]);
    rb_hip_vel   = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RB_Hip],   BodyCoM[RB_Hip]);
    rb_thigh_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RB_Thigh], BodyCoM[RB_Thigh]);
    rb_shank_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RB_Shank], BodyCoM[RB_Shank]);

    com_velocity = (BodyMass[Base]     * base_vel
                  + BodyMass[LF_Hip]   * lf_hip_vel
                  + BodyMass[LF_Thigh] * lf_thigh_vel
                  + BodyMass[LF_Shank] * lf_shank_vel
                  + BodyMass[LB_Hip]   * lb_hip_vel
                  + BodyMass[LB_Thigh] * lb_thigh_vel
                  + BodyMass[LB_Shank] * lb_shank_vel
                  + BodyMass[RF_Hip]   * rf_hip_vel
                  + BodyMass[RF_Thigh] * rf_thigh_vel
                  + BodyMass[RF_Shank] * rf_shank_vel
                  + BodyMass[RB_Hip]   * rb_hip_vel
                  + BodyMass[RB_Thigh] * rb_thigh_vel
                  + BodyMass[RB_Shank] * rb_shank_vel)/(BodyMass[Base] 
                  
                  + BodyMass[LF_Hip] + BodyMass[LF_Thigh] + BodyMass[LF_Shank]
                  + BodyMass[LB_Hip] + BodyMass[LB_Thigh] + BodyMass[LB_Shank]
                  + BodyMass[RF_Hip] + BodyMass[RF_Thigh] + BodyMass[RF_Shank]
                  + BodyMass[RB_Hip] + BodyMass[RB_Thigh] + BodyMass[RB_Shank]);

    return com_velocity;
}

Vec31<double> QuadrupedDynamicsModel::get_Base_Velocity_from_CoM(const Vector3d& CoM_vel){
    Vector3d base_velocity;

    Vector3d lf_hip_vel, lf_thigh_vel, lf_shank_vel;
    Vector3d lb_hip_vel, lb_thigh_vel, lb_shank_vel;
    Vector3d rf_hip_vel, rf_thigh_vel, rf_shank_vel;
    Vector3d rb_hip_vel, rb_thigh_vel, rb_shank_vel;

    lf_hip_vel   = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LF_Hip],   BodyCoM[LF_Hip]);
    lf_thigh_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LF_Thigh], BodyCoM[LF_Thigh]);
    lf_shank_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LF_Shank], BodyCoM[LF_Shank]);
    lb_hip_vel   = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LB_Hip],   BodyCoM[LB_Hip]);
    lb_thigh_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LB_Thigh], BodyCoM[LB_Thigh]);
    lb_shank_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LB_Shank], BodyCoM[LB_Shank]);
    rf_hip_vel   = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RF_Hip],   BodyCoM[RF_Hip]);
    rf_thigh_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RF_Thigh], BodyCoM[RF_Thigh]);
    rf_shank_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RF_Shank], BodyCoM[RF_Shank]);
    rb_hip_vel   = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RB_Hip],   BodyCoM[RB_Hip]);
    rb_thigh_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RB_Thigh], BodyCoM[RB_Thigh]);
    rb_shank_vel = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RB_Shank], BodyCoM[RB_Shank]);

    base_velocity = ((BodyMass[Base] 
                  + BodyMass[LF_Hip] + BodyMass[LF_Thigh] + BodyMass[LF_Shank]
                  + BodyMass[LB_Hip] + BodyMass[LB_Thigh] + BodyMass[LB_Shank]
                  + BodyMass[RF_Hip] + BodyMass[RF_Thigh] + BodyMass[RF_Shank]
                  + BodyMass[RB_Hip] + BodyMass[RB_Thigh] + BodyMass[RB_Shank]) * CoM_vel

                  -(BodyMass[LF_Hip]     * lf_hip_vel
                    + BodyMass[LF_Thigh] * lf_thigh_vel
                    + BodyMass[LF_Shank] * lf_shank_vel
                    + BodyMass[LB_Hip]   * lb_hip_vel
                    + BodyMass[LB_Thigh] * lb_thigh_vel
                    + BodyMass[LB_Shank] * lb_shank_vel
                    + BodyMass[RF_Hip]   * rf_hip_vel
                    + BodyMass[RF_Thigh] * rf_thigh_vel
                    + BodyMass[RF_Shank] * rf_shank_vel
                    + BodyMass[RB_Hip]   * rb_hip_vel
                    + BodyMass[RB_Thigh] * rb_thigh_vel
                    + BodyMass[RB_Shank] * rb_shank_vel))/BodyMass[Base];

    return base_velocity;
}

const Vec31<double>& QuadrupedDynamicsModel::get_CoM_in_BaseFrame(const Vector3d& CoM_Pos){
    com_in_base = CalcBaseToBodyCoordinates(*quadmodel, Q, body_id[Base], CoM_Pos);
    return com_in_base;
}

void QuadrupedDynamicsModel::CoM6DJacobian(){
    _Jcom_pre = Jcom;
    _JCoM.setZero();
    CalcPointJacobian6D(*quadmodel, Q, body_id[Base], BodyCoM[Base], _JCoM);
    Jcom.topRows(3) = _JCoM.bottomRows(3);
    Jcom.bottomRows(3) = _JCoM.topRows(3);
}

void QuadrupedDynamicsModel::CoM6DJacobian_c_frame(){
    MatrixNd _JCoM_tmp = MatrixNd::Zero(6, quadmodel->qdot_size);

    _Jcom_pre_c_frame = Jcom_c_frame;
    _JCoM_c_frame.setZero();
    // CalcPointJacobian6D(*quadmodel, Q_c_frame, body_id[Base], BodyCoM[Base], _JCoM_c_frame);
    CalcPointJacobian6D(*quadmodel, Q, body_id[Base], BodyCoM[Base], _JCoM_tmp);

    // std::cerr << "_JCoM_c_frame size:" << _JCoM_c_frame.rows() << "\t" << _JCoM_c_frame.cols() << "\n";
    // std::cerr << "_JCoM_c_frame" << _JCoM_c_frame << "\n";
    // Jcom_c_frame.topRows(3) = _JCoM_c_frame.bottomRows(3);
    // Jcom_c_frame.bottomRows(3) = _JCoM_c_frame.topRows(3);
    Jcom_c_frame.topRows(3) = rotMat_world_to_c * _JCoM_tmp.bottomRows(3);// linear
    Jcom_c_frame.bottomRows(3) = rotMat_world_to_c * _JCoM_tmp.topRows(3);// angular
    // Jcom_c_frame.bottomRows(3) =  _JCoM_tmp.topRows(3);// angular
}

const DMat<double>& QuadrupedDynamicsModel::swingFootJacobian(size_t foot_id){
    _JSwingFoot.setZero();
    if(foot_id == legID::LF)
        CalcPointJacobian(*quadmodel, Q, body_id[LF_Shank], contact_point, _JSwingFoot);
    else if(foot_id == legID::LB)
        CalcPointJacobian(*quadmodel, Q, body_id[LB_Shank], contact_point, _JSwingFoot);
    else if(foot_id == legID::RF)
        CalcPointJacobian(*quadmodel, Q, body_id[RF_Shank], contact_point, _JSwingFoot);
    else if(foot_id == legID::RB)
        CalcPointJacobian(*quadmodel, Q, body_id[RB_Shank], contact_point, _JSwingFoot);
    return _JSwingFoot;
}

const DMat<double>& QuadrupedDynamicsModel::swingFootJacobian_c_frame(size_t foot_id){
    // _JSwingFoot_c_frame.setZero();
    // if(foot_id == legID::LF)
    //     CalcPointJacobian(*quadmodel, Q_c_frame, body_id[LF_Shank], contact_point, _JSwingFoot_c_frame);
    // else if(foot_id == legID::LB)
    //     CalcPointJacobian(*quadmodel, Q_c_frame, body_id[LB_Shank], contact_point, _JSwingFoot_c_frame);
    // else if(foot_id == legID::RF)
    //     CalcPointJacobian(*quadmodel, Q_c_frame, body_id[RF_Shank], contact_point, _JSwingFoot_c_frame);
    // else if(foot_id == legID::RB)
    //     CalcPointJacobian(*quadmodel, Q_c_frame, body_id[RB_Shank], contact_point, _JSwingFoot_c_frame);
    // return _JSwingFoot_c_frame;

    MatrixNd _JCoM_tmp = MatrixNd::Zero(3, quadmodel->qdot_size);
    _JSwingFoot_c_frame.setZero();
    if(foot_id == legID::LF)
        CalcPointJacobian(*quadmodel, Q, body_id[LF_Shank], contact_point, _JCoM_tmp);
    else if(foot_id == legID::LB)
        CalcPointJacobian(*quadmodel, Q, body_id[LB_Shank], contact_point, _JCoM_tmp);
    else if(foot_id == legID::RF)
        CalcPointJacobian(*quadmodel, Q, body_id[RF_Shank], contact_point, _JCoM_tmp);
    else if(foot_id == legID::RB)
        CalcPointJacobian(*quadmodel, Q, body_id[RB_Shank], contact_point, _JCoM_tmp);
    
    _JSwingFoot_c_frame = rotMat_world_to_c * _JCoM_tmp;

    return  _JSwingFoot_c_frame;
}


DVec<double> QuadrupedDynamicsModel::swingFootPosition (size_t foot_id){
    if(foot_id == legID::LF)
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LF_Shank], contact_point);
    else if(foot_id == legID::LB)
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LB_Shank], contact_point);
    else if(foot_id == legID::RF)
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RF_Shank], contact_point);
    else if(foot_id == legID::RB)
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RB_Shank], contact_point);
    return _foot_position;
}

DVec<double> QuadrupedDynamicsModel::swingFootPosition_c_frame (size_t foot_id){
    if(foot_id == legID::LF)
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, Q_c_frame, body_id[LF_Shank], contact_point);
    else if(foot_id == legID::LB)
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, Q_c_frame, body_id[LB_Shank], contact_point);
    else if(foot_id == legID::RF)
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, Q_c_frame, body_id[RF_Shank], contact_point);
    else if(foot_id == legID::RB)
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, Q_c_frame, body_id[RB_Shank], contact_point);
    return _foot_position;
}

DVec<float> QuadrupedDynamicsModel::swingFootPosition (size_t foot_id,const VectorNd &given_Q){
    bool update_kinematics = true;
    if(foot_id == legID::LF)
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, given_Q, body_id[LF_Shank], contact_point, update_kinematics);
    else if(foot_id == legID::LB)
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, given_Q, body_id[LB_Shank], contact_point, update_kinematics);
    else if(foot_id == legID::RF)
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, given_Q, body_id[RF_Shank], contact_point, update_kinematics);
    else if(foot_id == legID::RB)
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, given_Q, body_id[RB_Shank], contact_point, update_kinematics);
    return _foot_position.cast<float>();
}


DVec<double> QuadrupedDynamicsModel::swingFootVelocity (size_t foot_id){
    if(foot_id == legID::LF)
        _foot_velocity = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LF_Shank], contact_point);
    else if(foot_id == legID::LB)
        _foot_velocity = CalcPointVelocity(*quadmodel, Q, QDot, body_id[LB_Shank], contact_point);
    else if(foot_id == legID::RF)
        _foot_velocity = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RF_Shank], contact_point);
    else if(foot_id == legID::RB)
        _foot_velocity = CalcPointVelocity(*quadmodel, Q, QDot, body_id[RB_Shank], contact_point);
    return _foot_velocity;
}

DVec<double> QuadrupedDynamicsModel::swingFootVelocity_c_frame (size_t foot_id){
    if(foot_id == legID::LF)
        _foot_velocity = CalcPointVelocity(*quadmodel, Q_c_frame, QDot_c_frame, body_id[LF_Shank], contact_point);
    else if(foot_id == legID::LB)
        _foot_velocity = CalcPointVelocity(*quadmodel, Q_c_frame, QDot_c_frame, body_id[LB_Shank], contact_point);
    else if(foot_id == legID::RF)
        _foot_velocity = CalcPointVelocity(*quadmodel, Q_c_frame, QDot_c_frame, body_id[RF_Shank], contact_point);
    else if(foot_id == legID::RB)
        _foot_velocity = CalcPointVelocity(*quadmodel, Q_c_frame, QDot_c_frame, body_id[RB_Shank], contact_point);
    return _foot_velocity;
}

Vec31<double> QuadrupedDynamicsModel::hipPosition(size_t hip_id){//for motion plan
    if(hip_id == 0){
        Vector3d hip_in_base_lf(-0.177, 0.33, 0);
        hip_position = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[Base], hip_in_base_lf);
    }
    if(hip_id == 1){
        Vector3d hip_in_base_lb(-0.177, -0.33, 0);
        hip_position = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[Base], hip_in_base_lb);
    }
    if(hip_id == 2){
        Vector3d hip_in_base_rb(0.177, -0.33, 0);
        hip_position = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[Base], hip_in_base_rb);
    }
    if(hip_id == 3){
        Vector3d hip_in_base_rf(0.177, 0.33, 0);
        hip_position = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[Base], hip_in_base_rf);
    }

    return hip_position;
}

Vec31<double> QuadrupedDynamicsModel::hipVelocity(size_t hip_id){// for motion plan
    if(hip_id == 0){
        Vector3d hip_in_base_lf(-0.135, 0.33, 0);
        hip_velocity = CalcPointVelocity(*quadmodel, Q, QDot, body_id[Base], hip_in_base_lf);
    }
    if(hip_id == 1){
        Vector3d hip_in_base_lb(-0.135, -0.33, 0);
        hip_velocity = CalcPointVelocity(*quadmodel, Q, QDot, body_id[Base], hip_in_base_lb);
    }
    if(hip_id == 2){
        Vector3d hip_in_base_rb(0.135, -0.33, 0);
        hip_velocity = CalcPointVelocity(*quadmodel, Q, QDot, body_id[Base], hip_in_base_rb);
    }
    if(hip_id == 3){
        Vector3d hip_in_base_rf(0.135, 0.33, 0);
        hip_velocity = CalcPointVelocity(*quadmodel, Q, QDot, body_id[Base], hip_in_base_rf);
    }

    return hip_velocity;
}

Vec31<double> QuadrupedDynamicsModel::footPosition(size_t foot_id){
    if(foot_id == 0){//lf
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LF_Shank], contact_point);
    }
    if(foot_id == 1){//lb
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[LB_Shank], contact_point);
    }
    if(foot_id == 2){//rb
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RB_Shank], contact_point);
    }
    if(foot_id == 3){//rf
        _foot_position = CalcBodyToBaseCoordinates(*quadmodel, Q, body_id[RF_Shank], contact_point);
    }

    return _foot_position;
}


DMat<double> QuadrupedDynamicsModel::getCoM6DJacobian(){
    return Jcom;
}

DMat<double> QuadrupedDynamicsModel::getCoM6DJacobian_c_frame(){
    return Jcom_c_frame;
}

const DVec<double> & QuadrupedDynamicsModel::getCJDotQDot(){
    // ROS_INFO_STREAM("_FOOTJ: \n"<<_FootJ);
    // ROS_INFO_STREAM("_FOOTJ_PRE: \n"<<_FootJ_pre);
    // ROS_INFO_STREAM("_FOOTJ_dt: \n"<<(_FootJ - _FootJ_pre));
    // ROS_INFO_STREAM("_FOOTJ_dt/DURATION: \n"<<(_FootJ - _FootJ_pre)/duration);
    // ROS_INFO_STREAM("DURATION: \n"<<duration);
    // for(int i(0); i<_CJDotQDot.rows(); i++){
    //     cout<<"THIS IS CJ: "<< _CJDotQDot[i];
    // }
    return  _CJDotQDot;
}

const DVec<double> & QuadrupedDynamicsModel::getCoM6DJDotQDot(){
    _JCoMDotQDot = (Jcom - _Jcom_pre)/duration * QDot;

    return _JCoMDotQDot;
}

const DVec<double> & QuadrupedDynamicsModel::getCoM6DJDotQDot_c_frame(){
    _JCoMDotQDot_c_frame = (Jcom_c_frame - _Jcom_pre_c_frame)/duration * QDot_c_frame;

    return _JCoMDotQDot_c_frame;
}


void QuadrupedDynamicsModel::initParameters(){
    BodyMass[Base]     = 26.398;
    BodyMass[LF_Hip]   = 1.5767;
    BodyMass[LF_Thigh] = 3.0063;
    BodyMass[LF_Shank] = 0.54849;
    BodyMass[LB_Hip]   = 1.5767;
    BodyMass[LB_Thigh] = 3.0063;
    BodyMass[LB_Shank] = 0.54849;
    BodyMass[RF_Hip]   = 1.5767;
    BodyMass[RF_Thigh] = 3.0062;
    BodyMass[RF_Shank] = 0.54849;
    BodyMass[RB_Hip]   = 1.5767;
    BodyMass[RB_Thigh] = 3.0062;
    BodyMass[RB_Shank] = 0.54849;



    BodyCoM[Base]     << 0.005, 0.000413, 0.004;
    BodyCoM[LF_Hip]   << -0.00075689, -0.0089129, -5.0528E-05;
    BodyCoM[LF_Thigh] << -0.0025264, -0.022118, -0.0292;
    BodyCoM[LF_Shank] << 0.0075224, 2.5888E-05, -0.17586;
    BodyCoM[LB_Hip]   << 0.000757, -0.009, 5.053E-05;
    BodyCoM[LB_Thigh] << -0.0025264, -0.022118, -0.0292;
    BodyCoM[LB_Shank] << 0.0075224, 2.5888E-05, -0.17586;
    BodyCoM[RF_Hip]   << -0.000757, 0.009, 5.0528E-05;
    BodyCoM[RF_Thigh] << -0.0025266, 0.022119, -0.029194;
    BodyCoM[RF_Shank] << 0.0075224, 2.5888E-05, -0.17586;
    BodyCoM[RB_Hip]   << 0.00075689, 0.0089129, -5.0528E-05;
    BodyCoM[RB_Thigh] << -0.0025266, 0.022119, -0.029194;
    BodyCoM[RB_Shank] << 0.0075224, 2.589E-05, -0.17586;



    BodyInertia[Base] << 0.1632, -5.2136E-05, -0.00019406,
                         -5.2136E-05, 0.40648, 8.2679E-05,
                         -0.00019406, 8.2679E-05, 0.52596;

    BodyInertia[LF_Hip] << 0.0012141, 6.1989E-06, 2.8652E-06,
                           6.1989E-06, 0.0016912, 3.7E-06,
                           2.8652E-06, 3.7E-06, 0.0013431;

    BodyInertia[LF_Thigh] << 0.0058102, 4.2848E-05, -3.0437E-06,
                             4.2848E-05, 0.0080686, -6.889E-05,
                             -3.0437E-06, -6.889E-05, 0.003874;

    BodyInertia[LF_Shank] << 0.0057, 1.16E-07, 0.000153,
                             1.16E-07, 0.006, -1.12E-06,
                             0.000153, -1.12E-06, 0.00029;

    BodyInertia[LB_Hip] << 0.00121, -6.2E-06, 2.9E-06,
                           -6.2E-06, 0.0017, -3.7E-06,
                           2.9E-06, -3.7E-06, 0.00134;

    BodyInertia[LB_Thigh] << 0.0058102, 4.2848E-05, -3.0433E-06,
                             4.2848E-05, 0.008, -6.8889E-05,
                             -3.0433E-06, -6.8889E-05, 0.003874;

    BodyInertia[LB_Shank] << 0.0057, 1.16E-07, 0.000153,
                             1.16E-07, 0.006, -1.12E-06,
                             0.000153, -1.12E-06, 0.00029;

    BodyInertia[RF_Hip] << 0.0012141, -6.1989E-06, -2.8652E-06,
                           -6.1989E-06, 0.0016912, 3.7E-06,
                           -2.8652E-06, 3.7E-06, 0.0013431;

    BodyInertia[RF_Thigh] << 0.00581, -4.2781E-05, -3.0682E-06,
                             -4.2781E-05, 0.0080684, 6.8812E-05,
                             -3.0682E-06, 6.8812E-05, 0.0038739;

    BodyInertia[RF_Shank] << 0.0057, 1.16E-07, 0.000153,
                             1.16E-07, 0.006, -1.12E-06,
                             0.000153, -1.12E-06, 0.00029;

    BodyInertia[RB_Hip] << 0.0012141, 6.1989E-06, -2.8652E-06,
                           6.1989E-06, 0.0016912, -3.7E-06,
                           -2.8652E-06, -3.7E-06, 0.0013431;

    BodyInertia[RB_Thigh] << 0.00581, -4.2781E-05, -3.0681E-06,
                             -4.2781E-05, 0.0080684, 6.8812E-05,
                             -3.0681E-06, 6.8812E-05, 0.0038739;

    BodyInertia[RB_Shank] << 0.0056981, 1.1608E-07, 0.00015292,
                             1.1608E-07, 0.0059026, -1.1243E-06,
                             0.00015292, -1.1243E-06, 0.00028822;



    Trans[FloatBase].R.setIdentity();
    Trans[FloatBase].Tr.setZero();

    Trans[LF_HipX].R.setIdentity();
    Trans[LF_HipX].Tr << 0.292, 0.08, 0;

    Trans[LF_HipY].R.setIdentity();
    Trans[LF_HipY].Tr << 0, 0.12325, 0;

    Trans[LF_Knee].R.setIdentity();
    Trans[LF_Knee].Tr << 0, 0, -0.3;

    Trans[LB_HipX].R.setIdentity();
    Trans[LB_HipX].Tr << -0.292, 0.08, 0;
    
    Trans[LB_HipY].R.setIdentity();
    Trans[LB_HipY].Tr << 0, 0.12325, 0;

    Trans[LB_Knee].R.setIdentity();
    Trans[LB_Knee].Tr << 0, 0, -0.3;

    Trans[RF_HipX].R.setIdentity();
    Trans[RF_HipX].Tr << 0.292, -0.08, 0;

    Trans[RF_HipY].R.setIdentity();
    Trans[RF_HipY].Tr << 0, -0.12325, 0;

    Trans[RF_Knee].R.setIdentity();
    Trans[RF_Knee].Tr << 0, 0, -0.3;
    
    Trans[RB_HipX].R.setIdentity();
    Trans[RB_HipX].Tr << -0.292, -0.08, 0;

    Trans[RB_HipY].R.setIdentity();
    Trans[RB_HipY].Tr << 0, -0.12325, 0;

    Trans[RB_Knee].R.setIdentity();
    Trans[RB_Knee].Tr << 0, 0, -0.3;

    //contact related
    contact_point << 0, 0, -0.33;
}


void QuadrupedDynamicsModel::generate(){
    Body Base_body = Body(BodyMass[Base], BodyCoM[Base], BodyInertia[Base]);
    Body LF_Hip_body = Body(BodyMass[LF_Hip], BodyCoM[LF_Hip], BodyInertia[LF_Hip]);
    Body LF_Thigh_body = Body(BodyMass[LF_Thigh], BodyCoM[LF_Thigh], BodyInertia[LF_Thigh]);
    Body LF_Shank_body = Body(BodyMass[LF_Shank], BodyCoM[LF_Shank], BodyInertia[LF_Shank]);
    Body LB_Hip_body = Body(BodyMass[LB_Hip], BodyCoM[LB_Hip], BodyInertia[LB_Hip]);
    Body LB_Thigh_body = Body(BodyMass[LB_Thigh], BodyCoM[LB_Thigh], BodyInertia[LB_Thigh]);
    Body LB_Shank_body = Body(BodyMass[LB_Shank], BodyCoM[LB_Shank], BodyInertia[LB_Shank]);
    Body RF_Hip_body = Body(BodyMass[RF_Hip], BodyCoM[RF_Hip], BodyInertia[RF_Hip]);
    Body RF_Thigh_body = Body(BodyMass[RF_Thigh], BodyCoM[RF_Thigh], BodyInertia[RF_Thigh]);
    Body RF_Shank_body = Body(BodyMass[RF_Shank], BodyCoM[RF_Shank], BodyInertia[RF_Shank]);
    Body RB_Hip_body = Body(BodyMass[RB_Hip], BodyCoM[RB_Hip], BodyInertia[RB_Hip]);
    Body RB_Thigh_body = Body(BodyMass[RB_Thigh], BodyCoM[RB_Thigh], BodyInertia[RB_Thigh]);
    Body RB_Shank_body = Body(BodyMass[RB_Shank], BodyCoM[RB_Shank], BodyInertia[RB_Shank]);

    Joint FloatBase_joint = Joint(JointTypeFloatingBase);
    Joint LF_HipX_joint = Joint(SpatialVector (1., 0., 0., 0., 0., 0.));
    Joint LF_HipY_joint = Joint(SpatialVector (0., -1., 0., 0., 0., 0.));
    Joint LF_Knee_joint = Joint(SpatialVector (0., -1., 0., 0., 0., 0.));
    Joint LB_HipX_joint = Joint(SpatialVector (1., 0., 0., 0., 0., 0.));
    Joint LB_HipY_joint = Joint(SpatialVector (0., -1., 0., 0., 0., 0.));
    Joint LB_Knee_joint = Joint(SpatialVector (0., -1., 0., 0., 0., 0.));
    Joint RF_HipX_joint = Joint(SpatialVector (-1., 0., 0., 0., 0., 0.));
    Joint RF_HipY_joint = Joint(SpatialVector (0., -1., 0., 0., 0., 0.));
    Joint RF_Knee_joint = Joint(SpatialVector (0., -1., 0., 0., 0., 0.));
    Joint RB_HipX_joint = Joint(SpatialVector (-1., 0., 0., 0., 0., 0.));
    Joint RB_HipY_joint = Joint(SpatialVector (0., -1., 0., 0., 0., 0.));
    Joint RB_Knee_joint = Joint(SpatialVector (0., -1., 0., 0., 0., 0.));


    if (!Addons::URDFReadFromFile (
                "/home/yjy/nuclear_pro/x20_sim_20230312/MPC_WBC_SIM/ocs2_ws/src/X20/urdf/X20_rsm.urdf", quadmodel, true, false)) {
        std::cerr << "Error loading model aliengo.urdf" << std::endl;
        abort();
    }
    const char* link_name[] =
    {
        "base",
        "LF_HIP","LF_THIGH","LF_SHANK",
        "LH_HIP","LH_THIGH","LH_SHANK",
        "RF_HIP","RF_THIGH","RF_SHANK",
        "RH_HIP","RH_THIGH","RH_SHANK"
    };
// Print link table
//     int linklist_len;
//     linklist_len = sizeof(link_name)/sizeof(link_name[0]);
//     cout << "link_num: " << linklist_len << endl;
//     vector<const char*> link_name_list(link_name, link_name + linklist_len);
//     vector<int> body_id_list;
//     int body_id;
//     for (int i(0); i < link_name_list.size(); ++i) {
//         body_id = quadmodel->GetBodyId(link_name_list[i]);
//         body_id_list.push_back(body_id);
//     }
//     for (int i(0); i < body_id_list.size(); ++i) {
//         cout << link_name_list[i] << ": " << body_id_list[i] << endl;
//     }
//   abort();
    body_id[Base] = quadmodel->GetBodyId(link_name[0]);
    body_id[LF_Hip] = quadmodel->GetBodyId(link_name[1]);
    body_id[LF_Thigh] = quadmodel->GetBodyId(link_name[2]);
    body_id[LF_Shank] = quadmodel->GetBodyId(link_name[3]);

    body_id[LB_Hip] = quadmodel->GetBodyId(link_name[4]);
    body_id[LB_Thigh] = quadmodel->GetBodyId(link_name[5]);
    body_id[LB_Shank] = quadmodel->GetBodyId(link_name[6]);

    body_id[RF_Hip] = quadmodel->GetBodyId(link_name[7]);
    body_id[RF_Thigh] = quadmodel->GetBodyId(link_name[8]);
    body_id[RF_Shank] = quadmodel->GetBodyId(link_name[9]);

    body_id[RB_Hip] = quadmodel->GetBodyId(link_name[10]);
    body_id[RB_Thigh] = quadmodel->GetBodyId(link_name[11]);
    body_id[RB_Shank] = quadmodel->GetBodyId(link_name[12]);
    quadmodel->gravity = Vector3d(0., 0., -9.81);

    // //Base
    // body_id[Base] = quadmodel->AddBody(0, SpatialTransform(Trans[FloatBase].R, Trans[FloatBase].Tr), FloatBase_joint, Base_body, "Base");
    
    // //LF
    // body_id[LF_Hip] = quadmodel->AddBody(body_id[Base], SpatialTransform(Trans[LF_HipX].R, Trans[LF_HipX].Tr), LF_HipX_joint, LF_Hip_body, "LF_Hip");
    // body_id[LF_Thigh] = quadmodel->AppendBody(SpatialTransform(Trans[LF_HipY].R, Trans[LF_HipY].Tr), LF_HipY_joint, LF_Thigh_body, "LF_Thigh");
    // body_id[LF_Shank] = quadmodel->AppendBody(SpatialTransform(Trans[LF_Knee].R, Trans[LF_Knee].Tr), LF_Knee_joint, LF_Shank_body, "LF_Shank");
    
    // //LB
    // body_id[LB_Hip] = quadmodel->AddBody(body_id[Base], SpatialTransform(Trans[LB_HipX].R, Trans[LB_HipX].Tr), LB_HipX_joint, LB_Hip_body, "LB_Hip");
    // body_id[LB_Thigh] = quadmodel->AppendBody(SpatialTransform(Trans[LB_HipY].R, Trans[LB_HipY].Tr), LB_HipY_joint, LB_Thigh_body, "LB_Thigh");
    // body_id[LB_Shank] = quadmodel->AppendBody(SpatialTransform(Trans[LB_Knee].R, Trans[LB_Knee].Tr), LB_Knee_joint, LB_Shank_body, "LB_Shank");
    
    // //RF
    // body_id[RF_Hip] = quadmodel->AddBody(body_id[Base], SpatialTransform(Trans[RF_HipX].R, Trans[RF_HipX].Tr), RF_HipX_joint, RF_Hip_body, "RF_Hip");
    // body_id[RF_Thigh] = quadmodel->AppendBody(SpatialTransform(Trans[RF_HipY].R, Trans[RF_HipY].Tr), RF_HipY_joint, RF_Thigh_body, "RF_Thigh");
    // body_id[RF_Shank] = quadmodel->AppendBody(SpatialTransform(Trans[RF_Knee].R, Trans[RF_Knee].Tr), RF_Knee_joint, RF_Shank_body, "RF_Shank");
    
    // //RB
    // body_id[RB_Hip] = quadmodel->AddBody(body_id[Base], SpatialTransform(Trans[RB_HipX].R, Trans[RB_HipX].Tr), RB_HipX_joint, RB_Hip_body, "RB_Hip");
    // body_id[RB_Thigh] = quadmodel->AppendBody(SpatialTransform(Trans[RB_HipY].R, Trans[RB_HipY].Tr), RB_HipY_joint, RB_Thigh_body, "RB_Thigh");
    // body_id[RB_Shank] = quadmodel->AppendBody(SpatialTransform(Trans[RB_Knee].R, Trans[RB_Knee].Tr), RB_Knee_joint, RB_Shank_body, "RB_Shank");
}



