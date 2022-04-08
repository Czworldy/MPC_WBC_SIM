#include "CoMAngularMotion.h"

//TODO：未测试函数计算是否正确

template<typename T>
CoMAngularMotion<T>::CoMAngularMotion(QuadrupedDynamicsModel* model)
     :Task<T>(), _robot_sys(model){}

template<typename T>
CoMAngularMotion<T>::~CoMAngularMotion(){}


template<typename T>
bool CoMAngularMotion<T>::UpdateTask(const DVec<T>& pos_des, 
                                     const DVec<T>& vel_des,
                                     const DVec<T>& acc_des,
                                     const Vec41<T>& contact_state){
    des_quat_in_frame_c_ = _robot_sys->quat_world_to_c.cast<T>() * rpyTOquaternion(pos_des[0], pos_des[1], pos_des[2]);
    // des_quat_in_frame_c_ = rpyTOquaternion(pos_des[0], pos_des[1], pos_des[2]);
    omega_d_ = _robot_sys->rotMat_world_to_c.cast<T>() * vel_des;
    acc_d_   = _robot_sys->rotMat_world_to_c.cast<T>() * acc_des;
    contactState = contact_state;

    Kp = user_p_.Kp_ori;
    Kd = user_p_.Kd_ori;

    if(!contactState[legID::LF]){
        Kp = user_p_.Kp_ori_lf;
        Kd = user_p_.Kd_ori_lf;
    }
    if(!contactState[legID::LB]){
        Kp = user_p_.Kp_ori_lb;
        Kd = user_p_.Kd_ori_lb;
    }
    if(!contactState[legID::RF]){
        Kp = user_p_.Kp_ori_rf;
        Kd = user_p_.Kd_ori_rf;
    }
    if(!contactState[legID::RB]){
        Kp = user_p_.Kp_ori_rb;
        Kd = user_p_.Kd_ori_rb;
    }
    
    Update_size();
    Update_A();
    Update_b();

    return true;
}

template<typename T>
bool CoMAngularMotion<T>::Update_size(){
    TK::dim_config_ = JYPro::dim_config;
    TK::dim_contact_ = _robot_sys->num_contact;
    TK::dim_task_eq_ = 3;
    TK::dim_task_ineq_ = 0; 
    TK::dim_optVar_ = TK::dim_config_+ 3*  TK::dim_contact_;

    TK::A_ = DMat<T>::Zero(TK::dim_task_eq_, TK::dim_optVar_);
    TK::b_ = DVec<T>::Zero(TK::dim_task_eq_);
    TK::D_ = DMat<T>::Zero(TK::dim_task_ineq_, TK::dim_optVar_);
    TK::f_ = DVec<T>::Zero(TK::dim_task_ineq_);

    return true;
}

template<typename T>
bool CoMAngularMotion<T>::Update_A(){
    DMat<T> A_in_frame_c = DMat<T>::Zero(TK::dim_task_eq_, TK::dim_optVar_);
    A_in_frame_c.leftCols(TK::dim_config_) = _robot_sys->getCoM6DJacobian_c_frame().bottomRows(3).cast<T>();
    TK::A_ = A_in_frame_c * _robot_sys->rotMatForTracking.cast<T>();
    // TK::A_ = A_in_frame_c; // yjy:不能这样写

    return true;
}

template<typename T>
bool CoMAngularMotion<T>::Update_b(){
    cur_quat_in_frame_c_inv_.x() = - _robot_sys->Q_c_frame[3];
    cur_quat_in_frame_c_inv_.y() = - _robot_sys->Q_c_frame[4];
    cur_quat_in_frame_c_inv_.z() = - _robot_sys->Q_c_frame[5];
    cur_quat_in_frame_c_inv_.w() = _robot_sys->Q_c_frame[18];

    Eigen::Quaternion<T> err_quat_in_frame_c = des_quat_in_frame_c_ * cur_quat_in_frame_c_inv_;
    if (err_quat_in_frame_c.w() < 0.) {
        err_quat_in_frame_c.w() *= (-1.);
    }

    Vec31<T> ori_err_so3;
    quaternionToso3(err_quat_in_frame_c, ori_err_so3);

    omega_cur_in_frame_c_[0] = _robot_sys->QDot_c_frame[3];
    omega_cur_in_frame_c_[1] = _robot_sys->QDot_c_frame[4];
    omega_cur_in_frame_c_[2] = _robot_sys->QDot_c_frame[5];

    TK::b_[0] = Kd[0]*(omega_d_[0] - omega_cur_in_frame_c_[0]) + Kp[0] * ori_err_so3[0] ; //- JDotQDot[3];
    TK::b_[1] = Kd[1]*(omega_d_[1] - omega_cur_in_frame_c_[1]) + Kp[1] * ori_err_so3[1] ; //- JDotQDot[4];
    TK::b_[2] = Kd[2]*(omega_d_[2] - omega_cur_in_frame_c_[2]) + Kp[2] * ori_err_so3[2] ; //- JDotQDot[5];

    return true;
}

// template<typename T>
// bool CoMAngularMotion<T>::Update_b(){
//     DVec<T> Q_CoM, QDot_CoM, JDotQDot;
//     Mat3<T> rot_cur, rot_des, rot_err;
//     Q_CoM = _robot_sys->Q.cast<T>();
//     QDot_CoM = _robot_sys->QDot.cast<T>();
//     JDotQDot = _robot_sys->getCoM6DJDotQDot().cast<T>();
//     //ROS_INFO_STREAM("JDotQDot: \n" << JDotQDot);


//     Vec31<T> roll_point;
//     T start_point(0), final_point(0.2), final_time(10*400);
//     TrajectoryPlan(start_point,final_point, final_time, iter, roll_point);

//     rot_cur = rpyTORotateMat(Q_CoM[3],Q_CoM[4], Q_CoM[5]);
//     rot_des = rpyTORotateMat(roll_point[0],angle_d_[1],angle_d_[2]);
//     rot_err =  rot_des * rot_cur.transpose();
    
//     TK::b_[0] = roll_point[2] + user_p_.Kd_ori[0]*(roll_point[1] - QDot_CoM[3]) + user_p_.Kp_ori[0]* rot_err(2,1);// - JDotQDot[3];
//     TK::b_[1] = -user_p_.Kd_ori[1]*QDot_CoM[4] + user_p_.Kp_ori[1]* rot_err(0,2);// - JDotQDot[4];
//     TK::b_[2] = -user_p_.Kd_ori[2]*QDot_CoM[5] + user_p_.Kp_ori[2]* rot_err(1,0) ;//- JDotQDot[5];
//     iter++;
//     ROS_INFO_STREAM("roll_____________________: "<<Q_CoM[3] );
// }

template<typename T>
void CoMAngularMotion<T>::TaskPrint(){
    ROS_INFO("TASK_PRINT_COMANGULARMOTION");
}

template<typename T>
bool CoMAngularMotion<T>::UpdateTask(){
    ROS_INFO("COMANGULARMOTION ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");
}

template<typename T>
bool CoMAngularMotion<T>::UpdateTask(const DVec<T>& pos_des, 
                                     const DVec<T>& vel_des,
                                     const DVec<T>& acc_des){
    ROS_INFO("COMANGULARMOTION ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");                                                 
}

template<typename T>
bool CoMAngularMotion<T>::UpdateTask(const Vec31<T>* pos_des, 
                                     const Vec31<T>* vel_des,
                                     const Vec31<T>* acc_des,
                                     const Vec41<T>& contact_state){
    ROS_INFO("COMANGULARMOTION ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");    
}

template<typename T>
bool CoMAngularMotion<T>::Update_D(){}

template<typename T>
bool CoMAngularMotion<T>::Update_f(){ }


template class CoMAngularMotion<double>;
template class CoMAngularMotion<float>;































// #include "CoMAngularMotion.h"
// #include "utility.h"

// //TODO：未测试函数计算是否正确

// template<typename T>
// CoMAngularMotion<T>::CoMAngularMotion(QuadrupedDynamicsModel* model)
//      :Task<T>(), _robot_sys(model){}

// template<typename T>
// CoMAngularMotion<T>::~CoMAngularMotion(){}

// template<typename T>
// bool CoMAngularMotion<T>::UpdateTask(const Vec31<T>& angle_des,
//                                                                                       const UserParameter<T>& user_param){
//     angle_d_ = angle_des;
//     user_p_ = user_param;
    
//     Update_size();
//     Update_A();
//     Update_b();
// }

// template<typename T>
// bool CoMAngularMotion<T>::Update_size(){
//     TK::dim_config_ = JYPro::dim_config;
//     TK::dim_contact_ = _robot_sys->num_contact;
//     TK::dim_task_eq_ = 3;
//     TK::dim_task_ineq_ = 0; 
//     TK::dim_optVar_ = TK::dim_config_+ 3*  TK::dim_contact_;

//     TK::A_ = DMat<T>::Zero(TK::dim_task_eq_, TK::dim_optVar_);
//     TK::b_ = DVec<T>::Zero(TK::dim_task_eq_);
//     TK::D_ = DMat<T>::Zero(TK::dim_task_ineq_, TK::dim_optVar_);
//     TK::f_ = DVec<T>::Zero(TK::dim_task_ineq_);
// }

// template<typename T>
// bool CoMAngularMotion<T>::Update_A(){
//     DMat<T> CoMJacobian;
//     CoMJacobian = _robot_sys->getCoM6DJacobian().cast<T>();
//     TK::A_.leftCols(TK::dim_config_) = CoMJacobian.bottomRows(3);
// }

// template<typename T>
// bool CoMAngularMotion<T>::Update_b(){
//     DVec<T> Q_CoM, QDot_CoM, JDotQDot;
//     Mat3<T> rot_cur, rot_des, rot_err;
//     Q_CoM = _robot_sys->Q.cast<T>();
//     QDot_CoM = _robot_sys->QDot.cast<T>();
//     JDotQDot = _robot_sys->getCoM6DJDotQDot().cast<T>();
//     //ROS_INFO_STREAM("JDotQDot: \n" << JDotQDot);
//     rot_cur = rpyTORotateMat(Q_CoM[3],Q_CoM[4], Q_CoM[5]);
//     rot_des = rpyTORotateMat(angle_d_[0],angle_d_[1],angle_d_[2]);
//     rot_err =  rot_des * rot_cur.transpose();
    
//     TK::b_[0] = -user_p_.Kd_ori[0]*QDot_CoM[3] + user_p_.Kp_ori[0]* rot_err(2,1);// - JDotQDot[3];
//     TK::b_[1] = -user_p_.Kd_ori[1]*QDot_CoM[4] + user_p_.Kp_ori[1]* rot_err(0,2) ;//- JDotQDot[4];
//     TK::b_[2] = -user_p_.Kd_ori[2]*QDot_CoM[5] + user_p_.Kp_ori[2]* rot_err(1,0) ;//- JDotQDot[5];
// }

// template<typename T>
// void CoMAngularMotion<T>::TaskPrint(){
//     ROS_INFO("TASK_PRINT_COMANGULARMOTION");
// }

// template<typename T>
// bool CoMAngularMotion<T>::UpdateTask(){
//     ROS_INFO("COMANGULARMOTION ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");
// }

// template<typename T>
// bool CoMAngularMotion<T>::UpdateTask(const DVec<T>& pos_des, 
//                                                                                      const DVec<T>& vel_des,
//                                                                                      const DVec<T>& acc_des,
//                                                                                      const UserParameter<T> & user_param){
//     ROS_INFO("COMANGULARMOTION ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");
// }

// template<typename T>
// bool CoMAngularMotion<T>::UpdateTask(const UserParameter<T> & user_param){
//     ROS_INFO("COMANGULARMOTION ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");
// }

// template<typename T>
// bool CoMAngularMotion<T>::Update_D(){}

// template<typename T>
// bool CoMAngularMotion<T>::Update_f(){ }


// template class CoMAngularMotion<double>;
// template class CoMAngularMotion<float>;