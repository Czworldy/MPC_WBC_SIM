#include "WBC_CONTROL/wbc_ctrl/TaskSet/NoContactMotion.h"
#include "cppTypes.h"
#include <iostream>

//No contact motion
template<typename T>
NoContactMotion<T>::NoContactMotion(QuadrupedDynamicsModel* model)
     :Task<T>(), _robot_sys(model){}

template<typename T>
bool NoContactMotion<T>::UpdateTask(){
    Update_size();

    Update_A();
    Update_b();

    return true;
}

template<typename T>
bool NoContactMotion<T>::Update_size(){
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

template<typename T>
bool NoContactMotion<T>::Update_A(){
    TK::A_.leftCols(TK::dim_config_) = _robot_sys->getContactJacobian().cast<T>();

    return true;
}

template<typename T>
bool NoContactMotion<T>::Update_b(){
    TK::b_ = -_robot_sys->getCJDotQDot().cast<T>();
    // printf_STREAM("CJDOTQDOT: \n"<<_robot_sys->getCJDotQDot());
    // printf_STREAM("NO_CONTACT_MOTION_b: \n"<<TK::b_);
    // printf_STREAM("dim_contact_:"<<TK::dim_task_eq_);
    // for(int i(0); i<TK::b_.rows();i++){
    //     cout<<"THIS IS B"<<TK::b_[i]<<endl;
    // }

    return true;

}

template<typename T>
void NoContactMotion<T>::TaskPrint(){
    printf("TASK_PRINT_NOCONTACTMOTION");
}


template<typename T>
bool NoContactMotion<T>::UpdateTask(const DVec<T>& pos_des, 
                                    const DVec<T>& vel_des,
                                    const DVec<T>& acc_des){
    printf("NoContactMotion ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");return true;
}

template<typename T>
bool NoContactMotion<T>::UpdateTask(const DVec<T>& pos_des, 
                                    const DVec<T>& vel_des,
                                    const DVec<T>& acc_des,
                                    const Vec41<T>& contact_state){
printf("NoContactMotion ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");       return true;           
}

template<typename T>
bool NoContactMotion<T>::UpdateTask(const Vec31<T>* pos_des, 
                                    const Vec31<T>* vel_des,
                                    const Vec31<T>* acc_des,
                                    const Vec41<T>& contact_state){
printf("NoContactMotion ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");      return true;             
}

template<typename T>
bool NoContactMotion<T>::Update_D(){return true;}

template<typename T>
bool NoContactMotion<T>::Update_f(){return true;}

template class NoContactMotion<double>;
template class NoContactMotion<float>;