#include "WBC_CONTROL/wbc_ctrl/TaskSet/SwingLegMotion.h"
#include <vector>

template<typename T>
SwingLegMotion<T>::SwingLegMotion(QuadrupedDynamicsModel* model)
     :Task<T>(), _robot_sys(model){
    
}

template<typename T>
SwingLegMotion<T>::~SwingLegMotion(){}

template<typename T>
bool SwingLegMotion<T>::UpdateTask(const Vec31<T>* pos_des, 
                                   const Vec31<T>* vel_des,
                                   const Vec31<T>* acc_des,
                                   const Vec41<T>& contact_state){
    for(int i(0); i<4; i++){
        pos_d_[i] = _robot_sys->rotMat_world_to_c.cast<T>() * (pos_des[i] - _robot_sys->xyz_c_to_world.cast<T>());
        vel_d_[i] = _robot_sys->rotMat_world_to_c.cast<T>() * vel_des[i];
        acc_d_[i] = _robot_sys->rotMat_world_to_c.cast<T>() * acc_des[i];
    }
    contactState = contact_state;

    num_swing = 0;
    if(!contactState[legID::LF])
    {
        num_swing++;
        Kp_lf = user_p_.Kp_foot_lf;
        Kd_lf = user_p_.Kd_foot_lf;
    }
    if(!contactState[legID::LB])
    {
        num_swing++;
        Kp_lh = user_p_.Kp_foot_lb;
        Kd_lh = user_p_.Kd_foot_lb;
    }
    if(!contactState[legID::RF])
    {
        num_swing++;
        Kp_rf = user_p_.Kp_foot_rf;
        Kd_rf = user_p_.Kd_foot_rf;
    }
    if(!contactState[legID::RB])
    {
        num_swing++;
        Kp_rh = user_p_.Kp_foot_rb;
        Kd_rh = user_p_.Kd_foot_rb;
    }

    Update_size();
    Update_A();
    Update_b();   

    return true;
}

template<typename T>
bool SwingLegMotion<T>::Update_size(){
    TK::dim_config_ = JYPro::dim_config;
    TK::dim_contact_ = _robot_sys->num_contact;
    TK::dim_task_eq_ = 3*num_swing;
    TK::dim_task_ineq_ = 0; 
    TK::dim_optVar_ = TK::dim_config_+ 3*  TK::dim_contact_;

    TK::A_ = DMat<T>::Zero(TK::dim_task_eq_, TK::dim_optVar_);
    TK::b_ = DVec<T>::Zero(TK::dim_task_eq_);
    TK::D_ = DMat<T>::Zero(TK::dim_task_ineq_, TK::dim_optVar_);
    TK::f_ = DVec<T>::Zero(TK::dim_task_ineq_);

    return true;
}

template<typename T>
bool SwingLegMotion<T>::Update_A(){
    std::vector<DMat<T>> FJacobi;

    if(!contactState[legID::LF])
    {
        FJacobi.push_back( _robot_sys->swingFootJacobian_c_frame(legID::LF).cast<T>());
    }
    if(!contactState[legID::LB])
    {
        FJacobi.push_back( _robot_sys->swingFootJacobian_c_frame(legID::LB).cast<T>());
    }
    if(!contactState[legID::RF])
    {
        FJacobi.push_back( _robot_sys->swingFootJacobian_c_frame(legID::RF).cast<T>());
    }
    if(!contactState[legID::RB])
    {
        FJacobi.push_back( _robot_sys->swingFootJacobian_c_frame(legID::RB).cast<T>());
    }
    
    DMat<T> A_in_frame_c = DMat<T>::Zero(TK::dim_task_eq_, TK::dim_optVar_);
    for(size_t i(0); i<FJacobi.size();i++) {
        A_in_frame_c.block(3*i, 0, 3, TK::dim_config_)  = FJacobi[i];
    }
    TK::A_ = A_in_frame_c * _robot_sys->rotMatForTracking.cast<T>();
    
    return true;
}

template<typename T>
bool SwingLegMotion<T>::Update_b(){

    std::vector<Vec31<T>> Fb;
    DVec<T> pos_c, vel_c;
    Vec31<T> b;

    if(!contactState[legID::LF])
    {
        pos_c = _robot_sys->swingFootPosition_c_frame(legID::LF).cast<T>();
        vel_c = _robot_sys->swingFootVelocity_c_frame(legID::LF).cast<T>();
        
        for(size_t i(0); i<3; i++){
            b[i] = acc_d_[legID::LF][i] + Kd_lf[i]*(vel_d_[legID::LF][i] - vel_c[i]) + Kp_lf[i]*(pos_d_[legID::LF][i] - pos_c[i]);
        }
        Fb.push_back(b);
    }

    if(!contactState[legID::LB])
    {
        pos_c = _robot_sys->swingFootPosition_c_frame(legID::LB).cast<T>();
        vel_c = _robot_sys->swingFootVelocity_c_frame(legID::LB).cast<T>();
        
        for(size_t i(0); i<3; i++) {
            b[i] = acc_d_[legID::LB][i] + Kd_lh[i]*(vel_d_[legID::LB][i] - vel_c[i]) + Kp_lh[i]*(pos_d_[legID::LB][i] - pos_c[i]);
        }
        Fb.push_back(b);
    }

    if(!contactState[legID::RF])
    {
        pos_c = _robot_sys->swingFootPosition_c_frame(legID::RF).cast<T>();
        vel_c = _robot_sys->swingFootVelocity_c_frame(legID::RF).cast<T>();
        
        for(size_t i(0); i<3; i++){
            b[i] = acc_d_[legID::RF][i] + Kd_rf[i]*(vel_d_[legID::RF][i] - vel_c[i]) + Kp_rf[i]*(pos_d_[legID::RF][i] - pos_c[i]);
        }        
        Fb.push_back(b);
    }

    if(!contactState[legID::RB])
    {
        pos_c = _robot_sys->swingFootPosition_c_frame(legID::RB).cast<T>();
        vel_c = _robot_sys->swingFootVelocity_c_frame(legID::RB).cast<T>();
        
        for(size_t i(0); i<3; i++) {
            b[i] = acc_d_[legID::RB][i] + Kd_rh[i]*(vel_d_[legID::RB][i] - vel_c[i]) + Kp_rh[i]*(pos_d_[legID::RB][i] - pos_c[i]);
        }
        Fb.push_back(b);
    }
   
    for(size_t i(0); i<Fb.size();i++) {
        TK::b_.segment(3*i,3)  = Fb[i];
    }

    return true;

}

template<typename T>
void SwingLegMotion<T>::TaskPrint(){
    printf("TASK_PRINT_SWINGLEGMOTION");
}

template<typename T>
bool SwingLegMotion<T>::UpdateTask(){
    printf("SwingLegMotion ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");return true;
}

template<typename T>
bool SwingLegMotion<T>::UpdateTask(const DVec<T>& pos_des, 
                                   const DVec<T>& vel_des,
                                   const DVec<T>& acc_des){
    printf("SwingLegMotion ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");return true;
}

template<typename T>
bool SwingLegMotion<T>::UpdateTask(const DVec<T>& pos_des, 
                                   const DVec<T>& vel_des,
                                   const DVec<T>& acc_des,
                                   const Vec41<T>& contact_state){
    printf("SwingLegMotion ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");return true;
}

template<typename T>
bool SwingLegMotion<T>::Update_D(){return true;}

template<typename T>
bool SwingLegMotion<T>::Update_f(){return true;}

template class SwingLegMotion<double>;
template class SwingLegMotion<float>;

