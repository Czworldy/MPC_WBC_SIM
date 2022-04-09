#include "CoMLinearMotion.h"

template<typename T>
CoMLinearMotion<T>::CoMLinearMotion(QuadrupedDynamicsModel* model)
     :Task<T>(), _robot_sys(model){}

template<typename T>
CoMLinearMotion<T>::~CoMLinearMotion(){}

template<typename T>
bool CoMLinearMotion<T>::UpdateTask(const DVec<T>& pos_des, 
                                    const DVec<T>& vel_des,
                                    const DVec<T>& acc_des,
                                    const Vec41<T>& contact_state){
    pos_d_ = _robot_sys->rotMat_world_to_c.cast<T>() * (pos_des - _robot_sys->xyz_c_to_world.cast<T>());
    vel_d_ = _robot_sys->rotMat_world_to_c.cast<T>() * vel_des;
    acc_d_ = _robot_sys->rotMat_world_to_c.cast<T>() * acc_des;
    contactState = contact_state;
    
    Kp = user_p_.Kp_body;
    Kd = user_p_.Kd_body;

    if(!contactState[legID::LF]){
        Kp = user_p_.Kp_body_lf;
        Kd = user_p_.Kd_body_lf;
    }
    if(!contactState[legID::LB]){
        Kp = user_p_.Kp_body_lb;
        Kd = user_p_.Kd_body_lb;
    }
    if(!contactState[legID::RF]){
        Kp = user_p_.Kp_body_rf;
        Kd = user_p_.Kd_body_rf;
    }
    if(!contactState[legID::RB]){
        Kp = user_p_.Kp_body_rb;
        Kd = user_p_.Kd_body_rb;
    }

    Update_size();
    Update_A();
    Update_b();
    return true;
}

template<typename T>
bool CoMLinearMotion<T>::Update_size(){
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


/* 
    TK::A_:size 3	30
    A_in_frame_c:size 3	18
*/
template<typename T>
bool CoMLinearMotion<T>::Update_A(){
    // DMat<T> A_in_frame_c = DMat<T>::Zero(TK::dim_task_eq_, TK::dim_optVar_);
    TK::A_.leftCols(TK::dim_config_)  = _robot_sys->getCoM6DJacobian_c_frame().topRows(3).cast<T>();
    // TK::A_ = A_in_frame_c * _robot_sys->rotMatForTracking.cast<T>();
    // TK::A_ = A_in_frame_c; // yjy:不能这样写
    return true;
}

template<typename T>
bool CoMLinearMotion<T>::Update_b(){   
    DVec<T> Q_CoM, QDot_CoM, JDotQDot;
    Q_CoM = _robot_sys->Q_c_frame.cast<T>();
    QDot_CoM = _robot_sys->QDot_c_frame.cast<T>();
    // JDotQDot = _robot_sys->getCoM6DJDotQDot_c_frame().cast<T>();
    for (size_t i(0); i<3; ++i){
        TK::b_[i] = acc_d_[i] + Kd[i]*(vel_d_[i] - QDot_CoM[i]) + Kp[i]*(pos_d_[i] - Q_CoM[i]);// - JDotQDot[i];
    }

    return true;
}

template<typename T>
void CoMLinearMotion<T>::TaskPrint(){
    ROS_INFO("TASK_PRINT_COMLINEARMOTION");
}

template<typename T>
bool CoMLinearMotion<T>::UpdateTask(){
    ROS_INFO("COMLINEARMOTION ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");
}

template<typename T>
bool CoMLinearMotion<T>::UpdateTask(const DVec<T>& pos_des, 
                                    const DVec<T>& vel_des,
                                    const DVec<T>& acc_des){
    ROS_INFO("COMLINEARMOTION ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");                                              
}

template<typename T>
bool CoMLinearMotion<T>::UpdateTask(const Vec31<T>* pos_des, 
                                    const Vec31<T>* vel_des,
                                    const Vec31<T>* acc_des,
                                    const Vec41<T>& contact_state){
    ROS_INFO("COMLINEARMOTION ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");                                              
}

template<typename T>
bool CoMLinearMotion<T>::Update_D(){}

template<typename T>
bool CoMLinearMotion<T>::Update_f(){}

template class CoMLinearMotion<double>;
template class CoMLinearMotion<float>;
