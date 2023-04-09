#include "WBC_CONTROL/wbc_ctrl/TaskSet/SwingLegJointMotion.h"
#include <vector>

template<typename T>
SwingLegJointMotion<T>::SwingLegJointMotion(QuadrupedDynamicsModel* model)
     :Task<T>(), _robot_sys(model){
    
}

template<typename T>
SwingLegJointMotion<T>::~SwingLegJointMotion(){}

template<typename T>
bool SwingLegJointMotion<T>::UpdateTask(const Vec31<T>* pos_des, 
                                        const Vec31<T>* vel_des,
                                        const Vec31<T>* acc_des,
                                        const Vec41<T>& contact_state){
    for(int i(0); i<4; i++){
        pos_d_[i] = pos_des[i];
        vel_d_[i] = vel_des[i];
        acc_d_[i] = acc_des[i];
    }
    contactState = contact_state;

    num_swing = 0;
    if(!contactState[legID::LF])
    {
        num_swing++;
        Kp_lf = user_p_.Kp_LegJoint_lf;
        Kd_lf = user_p_.Kd_LegJoint_lf;
    }
    if(!contactState[legID::LB])
    {
        num_swing++;
        Kp_lh = user_p_.Kp_LegJoint_lb;
        Kd_lh = user_p_.Kd_LegJoint_lb;
    }
    if(!contactState[legID::RF])
    {
        num_swing++;
        Kp_rf = user_p_.Kp_LegJoint_rf;
        Kd_rf = user_p_.Kd_LegJoint_rf;
    }
    if(!contactState[legID::RB])
    {
        num_swing++;
        Kp_rh = user_p_.Kp_LegJoint_rb;
        Kd_rh = user_p_.Kd_LegJoint_rb;
    }

    Update_size();
    Update_A();
    Update_b();   

    return true;
}

template<typename T>
bool SwingLegJointMotion<T>::Update_size(){
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
bool SwingLegJointMotion<T>::Update_A(){
    std::vector<DMat<T>> LegIdentity;

    if(!contactState[legID::LF])
    {   
        DMat<T> z = DMat<T>::Zero(3, TK::dim_config_);
        z.block(0, 6, 3, 3).setIdentity();
        LegIdentity.push_back(z);
    }
    if(!contactState[legID::LB])
    {
        DMat<T> z = DMat<T>::Zero(3, TK::dim_config_);
        z.block(0, 9, 3, 3).setIdentity();
        LegIdentity.push_back(z);
    }
    if(!contactState[legID::RF])
    {
        DMat<T> z = DMat<T>::Zero(3, TK::dim_config_);
        z.block(0, 12, 3, 3).setIdentity();
        LegIdentity.push_back(z);
    }
    if(!contactState[legID::RB])
    {
        DMat<T> z = DMat<T>::Zero(3, TK::dim_config_);
        z.block(0, 15, 3, 3).setIdentity();
        LegIdentity.push_back(z);
    }
    

    for(size_t i(0); i<LegIdentity.size();i++) {
        TK::A_.block(3*i, 0, 3, TK::dim_config_)  = LegIdentity[i];
    }

    
    
    return true;
}

template<typename T>
bool SwingLegJointMotion<T>::Update_b(){

    std::vector<Vec31<T>> Fb;
    DVec<T> pos_c, vel_c;
    Vec31<T> b;

    if(!contactState[legID::LF])
    {
        pos_c = _robot_sys->Q.segment(6, 3).cast<T>();
        vel_c = _robot_sys->QDot.segment(6, 3).cast<T>();
        
        for(size_t i(0); i<3; i++){
            b[i] = acc_d_[legID::LF][i] + Kd_lf[i]*(vel_d_[legID::LF][i] - vel_c[i]) + Kp_lf[i]*(pos_d_[legID::LF][i] - pos_c[i]);
        }
        Fb.push_back(b);
    }

    if(!contactState[legID::LB])
    {
        pos_c = _robot_sys->Q.segment(9, 3).cast<T>();
        vel_c = _robot_sys->QDot.segment(9, 3).cast<T>();
        
        for(size_t i(0); i<3; i++) {
            b[i] = acc_d_[legID::LB][i] + Kd_lh[i]*(vel_d_[legID::LB][i] - vel_c[i]) + Kp_lh[i]*(pos_d_[legID::LB][i] - pos_c[i]);
        }
        Fb.push_back(b);
    }

    if(!contactState[legID::RF])
    {
        pos_c = _robot_sys->Q.segment(12, 3).cast<T>();
        vel_c = _robot_sys->QDot.segment(12, 3).cast<T>();
        
        for(size_t i(0); i<3; i++){
            b[i] = acc_d_[legID::RF][i] + Kd_rf[i]*(vel_d_[legID::RF][i] - vel_c[i]) + Kp_rf[i]*(pos_d_[legID::RF][i] - pos_c[i]);
        }        
        Fb.push_back(b);
    }

    if(!contactState[legID::RB])
    {
        pos_c = _robot_sys->Q.segment(15, 3).cast<T>();
        vel_c = _robot_sys->QDot.segment(15, 3).cast<T>();;
        
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
void SwingLegJointMotion<T>::TaskPrint(){

}

template<typename T>
bool SwingLegJointMotion<T>::UpdateTask(){

}

template<typename T>
bool SwingLegJointMotion<T>::UpdateTask(const DVec<T>& pos_des, 
                                   const DVec<T>& vel_des,
                                   const DVec<T>& acc_des){

}

template<typename T>
bool SwingLegJointMotion<T>::UpdateTask(const DVec<T>& pos_des, 
                                   const DVec<T>& vel_des,
                                   const DVec<T>& acc_des,
                                   const Vec41<T>& contact_state){

}

template<typename T>
bool SwingLegJointMotion<T>::Update_D(){}

template<typename T>
bool SwingLegJointMotion<T>::Update_f(){}

template class SwingLegJointMotion<double>;
template class SwingLegJointMotion<float>;

