#include "TorqueLimits.h"

template <typename T>
TorqueLimits<T>::TorqueLimits(QuadrupedDynamicsModel* model)
     :Task<T>(), _robot_sys(model){}

template<typename T>
bool TorqueLimits<T>::UpdateTask(){

    Update_size();

    Update_D();
    Update_f();

    return true;
}

template<typename T>
bool TorqueLimits<T>::Update_size(){
    TK::dim_config_ = JYPro::dim_config;
    TK::dim_contact_ = _robot_sys->num_contact;
    TK::dim_task_eq_ = 0;
    TK::dim_task_ineq_ = 24; 
    TK::dim_optVar_ = TK::dim_config_+ 3*  TK::dim_contact_;

    TK::A_ = DMat<T>::Zero(TK::dim_task_eq_, TK::dim_optVar_);
    TK::b_ = DVec<T>::Zero(TK::dim_task_eq_);
    TK::D_ = DMat<T>::Zero(TK::dim_task_ineq_, TK::dim_optVar_);
    TK::f_ = DVec<T>::Zero(TK::dim_task_ineq_);

    return true;
}

template<typename T>
bool TorqueLimits<T>::Update_D(){
    Eigen::Matrix<T, 18, 18> MassMatrix;
    Eigen::Matrix<T, -1, 18, 0, 12, 18> ContactJacobian;
    MassMatrix = _robot_sys->getMassMatrix().cast<T>();
    ContactJacobian  = _robot_sys->getContactJacobian().cast<T>();

    TK::D_.topLeftCorner(TK::dim_config_-6, TK::dim_config_) = MassMatrix.bottomRows(TK::dim_config_-6);
    TK::D_.bottomLeftCorner(TK::dim_config_-6, TK::dim_config_) = - MassMatrix.bottomRows(TK::dim_config_-6);
    TK::D_.topRightCorner(TK::dim_config_-6, TK::dim_contact_*3) = - ContactJacobian.transpose().bottomRows(TK::dim_config_-6);
    TK::D_.bottomRightCorner(TK::dim_config_-6, TK::dim_contact_*3) = ContactJacobian.transpose().bottomRows(TK::dim_config_-6);

    return true;
}

template<typename T>
bool TorqueLimits<T>::Update_f(){
    DVec<T> NonlinearVector;
    NonlinearVector = _robot_sys->getNolinearEffect().cast<T>();

    TK::f_.topRows(TK::dim_config_-6) = user_p_.TauMax - NonlinearVector.bottomRows(TK::dim_config_-6);
    TK::f_.bottomRows(TK::dim_config_-6) = -user_p_.TauMin + NonlinearVector.bottomRows(TK::dim_config_-6);

    return true;
}

template<typename T>
void TorqueLimits<T>::TaskPrint(){
    ROS_INFO("TASK_PRINT_TORQUELIMITS");
}

template<typename T>
bool TorqueLimits<T>::UpdateTask(const DVec<T>& pos_des, 
                                 const DVec<T>& vel_des,
                                 const DVec<T>& acc_des){
    ROS_INFO("TorqueLimits ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");
}

template<typename T>
bool TorqueLimits<T>::UpdateTask(const DVec<T>& pos_des, 
                                 const DVec<T>& vel_des,
                                 const DVec<T>& acc_des,
                                 const Vec41<T>& contact_state){
    ROS_INFO("TorqueLimits ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");               
}

template<typename T>
bool TorqueLimits<T>::UpdateTask(const Vec31<T>* pos_des, 
                                 const Vec31<T>* vel_des,
                                 const Vec31<T>* acc_des,
                                 const Vec41<T>& contact_state){
    ROS_INFO("TorqueLimits ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");               
}

template<typename T>
bool TorqueLimits<T>::Update_A(){ }

template<typename T>
bool TorqueLimits<T>::Update_b(){ }


template class TorqueLimits<double>; 
template class TorqueLimits<float>; 