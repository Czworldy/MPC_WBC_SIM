#include "ContactForceMin.h"

template <typename T>
ContactForceMin<T>::ContactForceMin(QuadrupedDynamicsModel* model)
     :Task<T>(), _robot_sys(model){}

template <typename T>
ContactForceMin<T>::~ContactForceMin(){}

template <typename T>
bool ContactForceMin<T>::UpdateTask(){
    Update_size();
    Update_A();

    return true;
}

template <typename T>
bool ContactForceMin<T>::Update_size(){
    TK::dim_config_ = JYPro::dim_config;
    TK::dim_contact_ = _robot_sys->num_contact;
    TK::dim_task_eq_ = 3*TK::dim_contact_;
    TK::dim_task_ineq_ = 0; 
    TK::dim_optVar_ = TK::dim_config_+ 3*  TK::dim_contact_;

    TK::A_ = DMat<T>::Zero(TK::dim_task_eq_, TK::dim_optVar_);
    TK::b_ = DVec<T>::Zero(TK::dim_task_eq_);
    TK::D_ = DMat<T>::Zero(TK::dim_task_ineq_, TK::dim_optVar_);
    TK::f_ = DVec<T>::Zero(TK::dim_task_ineq_);

    return true;
}

template <typename T>
bool ContactForceMin<T>::Update_A(){
    TK::A_.rightCols(TK::dim_contact_*3).setIdentity();

    return true;
}

template <typename T>
void ContactForceMin<T>::TaskPrint(){
    ROS_INFO("TASK_PRINT_CONTACTFORCEMIN");   
}


template<typename T>
bool ContactForceMin<T>::UpdateTask(const DVec<T>& pos_des, 
                                    const DVec<T>& vel_des,
                                    const DVec<T>& acc_des){
    ROS_INFO("CONTACTFORCEMIN ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");
}

template<typename T>
bool ContactForceMin<T>::UpdateTask(const DVec<T>& pos_des, 
                                    const DVec<T>& vel_des,
                                    const DVec<T>& acc_des,
                                    const Vec41<T>& contact_state){
ROS_INFO("CONTACTFORCEMIN ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");                                   
}

template<typename T>
bool ContactForceMin<T>::UpdateTask(const Vec31<T>* pos_des, 
                                    const Vec31<T>* vel_des,
                                    const Vec31<T>* acc_des,
                                    const Vec41<T>& contact_state){
ROS_INFO("CONTACTFORCEMIN ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");                                   
}


template<typename T>
bool ContactForceMin<T>::Update_b(){ }

template<typename T>
bool ContactForceMin<T>::Update_D(){ }

template<typename T>
bool ContactForceMin<T>::Update_f(){ }

template class ContactForceMin<double>;
template class ContactForceMin<float>;