#include "WBC_CONTROL/wbc_ctrl/TaskSet/EoMTask.h"
#include "cppTypes.h"

//Floating base equations of motion
template<typename T>
EoMTask<T>::EoMTask(QuadrupedDynamicsModel* model)
     :Task<T>(), _robot_sys(model){}

template<typename T>
bool EoMTask<T>::UpdateTask(){
    Update_size();
    Update_A();
    Update_b();

    return true;
}

template<typename T>
bool EoMTask<T>::Update_size(){
    TK::dim_config_ = JYPro::dim_config;
    TK::dim_contact_ = _robot_sys->num_contact;
    TK::dim_task_eq_ = 6;
    TK::dim_task_ineq_ = 0; 
    TK::dim_optVar_ = TK::dim_config_+ 3*  TK::dim_contact_;

    TK::A_ = DMat<T>::Zero(TK::dim_task_eq_, TK::dim_optVar_);
    TK::b_ = DVec<T>::Zero(TK::dim_task_eq_);
    TK::D_ = DMat<T>::Zero(TK::dim_task_ineq_, TK::dim_optVar_);
    TK::f_ = DVec<T>::Zero(TK::dim_task_ineq_);

    return true;
}

template<typename T>
bool EoMTask<T>::Update_A(){
    TK::A_.leftCols(TK::dim_config_) = _robot_sys->getMassMatrix().cast<T>().topRows(TK::dim_task_eq_);
    TK::A_.rightCols(TK::dim_contact_*3) = - _robot_sys->getContactJacobian().cast<T>().transpose().topRows(TK::dim_task_eq_);

    return true;
}

template<typename T>
bool EoMTask<T>::Update_b(){
    TK::b_ = - _robot_sys->getNolinearEffect().cast<T>().topRows(TK::dim_task_eq_);

    return true;
}

template<typename T>
void EoMTask<T>::TaskPrint(){
    printf("TASK_PRINT_EOMTASK");
}


template<typename T>
bool EoMTask<T>::UpdateTask(const DVec<T>& pos_des, 
                            const DVec<T>& vel_des,
                            const DVec<T>& acc_des){
    printf("EoMTask ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");return true;
}

template<typename T>
bool EoMTask<T>::UpdateTask(const DVec<T>& pos_des, 
                            const DVec<T>& vel_des,
                            const DVec<T>& acc_des,
                            const Vec41<T>& contact_state){
printf("EoMTask ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");  return true;                          
}

template<typename T>
bool EoMTask<T>::UpdateTask(const Vec31<T>* pos_des, 
                            const Vec31<T>* vel_des,
                            const Vec31<T>* acc_des,
                            const Vec41<T>& contact_state){
printf("EoMTask ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");   return true;                         
}

template<typename T>
bool EoMTask<T>::Update_D(){return true; }

template<typename T>
bool EoMTask<T>::Update_f(){return true; }

template class EoMTask<double>;
template class EoMTask<float>;