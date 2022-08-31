#include "ContactForceLimits.h"

//Friction cone and lambda modulation

template <typename T>
ContactForceLimits<T>::ContactForceLimits(QuadrupedDynamicsModel* model)
     :Task<T>(), _robot_sys(model){}

template<typename T>
ContactForceLimits<T>::~ContactForceLimits(){}

template<typename T>
bool ContactForceLimits<T>::UpdateTask(){

    Update_size();
    Update_D();

    return true;
}

template<typename T>
bool ContactForceLimits<T>::Update_size(){
    TK::dim_config_ = JYPro::dim_config;
    TK::dim_contact_ = _robot_sys->num_contact;
    TK::dim_task_eq_ = 0;
    TK::dim_task_ineq_ = 4; 
    TK::dim_optVar_ = TK::dim_config_+ 3*  TK::dim_contact_;

    TK::A_ = DMat<T>::Zero(TK::dim_task_eq_, TK::dim_optVar_);
    TK::b_ = DVec<T>::Zero(TK::dim_task_eq_);
    TK::D_ = DMat<T>::Zero(TK::dim_task_ineq_, TK::dim_optVar_);
    TK::f_ = DVec<T>::Zero(TK::dim_task_ineq_);

    return true;
}

template<typename T>
bool ContactForceLimits<T>::Update_D(){
    friction_cone_.row(0) = (user_p_.HContact -user_p_.mu*user_p_.NContact).transpose();
    friction_cone_.row(1) = -(user_p_.HContact + user_p_.mu*user_p_.NContact).transpose();
    friction_cone_.row(2) = (user_p_.IContact - user_p_.mu*user_p_.NContact).transpose();
    friction_cone_.row(3) = -(user_p_.IContact + user_p_.mu*user_p_.NContact).transpose();

    for(int i = 0; i<TK::dim_contact_; i++){
        TK::D_.block(0, TK::dim_config_+i*3, 4, 3) = friction_cone_;
    }

    return true;
}

template<typename T>
void ContactForceLimits<T>::TaskPrint(){
    ROS_INFO("TASK_PRINT_CONTACTFORCELIMITS");
}

template<typename T>
bool ContactForceLimits<T>::UpdateTask(const DVec<T>& pos_des, 
                                       const DVec<T>& vel_des,
                                       const DVec<T>& acc_des){
    ROS_INFO("ContactForceLimits ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");
}

template<typename T>
bool ContactForceLimits<T>::UpdateTask(const DVec<T>& pos_des, 
                                       const DVec<T>& vel_des,
                                       const DVec<T>& acc_des,
                                       const Vec41<T>& contact_state){
    ROS_INFO("ContactForceLimits ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");                                      
}

template<typename T>
bool ContactForceLimits<T>::UpdateTask(const Vec31<T>* pos_des, 
                                       const Vec31<T>* vel_des,
                                       const Vec31<T>* acc_des,
                                       const Vec41<T>& contact_state){
    ROS_INFO("ContactForceLimits ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");                                      
}

template<typename T>
bool ContactForceLimits<T>::Update_f(){}

template<typename T>
bool ContactForceLimits<T>::Update_A(){}

template<typename T>
bool ContactForceLimits<T>::Update_b(){}

template class ContactForceLimits<double>;
template class ContactForceLimits<float>;