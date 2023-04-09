#include "WBC_CONTROL/wbc_ctrl/TaskSet/ContactForceLimits.h"

//Friction cone and lambda modulation

template <typename T>
ContactForceLimits<T>::ContactForceLimits(QuadrupedDynamicsModel* model)
     :Task<T>(), _robot_sys(model){}

template<typename T>
ContactForceLimits<T>::~ContactForceLimits(){}

template<typename T>
bool ContactForceLimits<T>::UpdateTask(const Eigen::Quaternion<T>& terrainOri) {

    terrainRotMat_ = terrainOri.toRotationMatrix();
    // std::cout << "terrainRotMat_ = \n" << terrainRotMat_ << std::endl;
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

    Mat13<T> HContact, NContact, IContact;
    HContact << terrainRotMat_(0,0), terrainRotMat_(1,0), terrainRotMat_(2,0);
    NContact << terrainRotMat_(0,2), terrainRotMat_(1,2), terrainRotMat_(2,2);
    IContact << terrainRotMat_(0,1), terrainRotMat_(1,1), terrainRotMat_(2,1);

    // HContact << 1,0,0;
    // NContact << 0,0,1;
    // IContact << 0,1,0;

    friction_cone_.row(0) =  (HContact - user_p_.mu * NContact).transpose();
    friction_cone_.row(1) = -(HContact + user_p_.mu * NContact).transpose();
    friction_cone_.row(2) =  (IContact - user_p_.mu * NContact).transpose();
    friction_cone_.row(3) = -(IContact + user_p_.mu * NContact).transpose();

    for(int i = 0; i<TK::dim_contact_; i++){
        TK::D_.block(0, TK::dim_config_+i*3, 4, 3) = friction_cone_;
    }

    return true;
}

template<typename T>
void ContactForceLimits<T>::TaskPrint(){
    //ROS_INFO("TASK_PRINT_CONTACTFORCELIMITS");
}

template<typename T>
bool ContactForceLimits<T>::UpdateTask(const DVec<T>& pos_des, 
                                       const DVec<T>& vel_des,
                                       const DVec<T>& acc_des){
    //ROS_INFO("ContactForceLimits ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");
}

template<typename T>
bool ContactForceLimits<T>::UpdateTask(const DVec<T>& pos_des, 
                                       const DVec<T>& vel_des,
                                       const DVec<T>& acc_des,
                                       const Vec41<T>& contact_state){
    //ROS_INFO("ContactForceLimits ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");                                      
}

template<typename T>
bool ContactForceLimits<T>::UpdateTask(const Vec31<T>* pos_des, 
                                       const Vec31<T>* vel_des,
                                       const Vec31<T>* acc_des,
                                       const Vec41<T>& contact_state){
    //ROS_INFO("ContactForceLimits ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");                                      
}

template<typename T>
bool ContactForceLimits<T>::UpdateTask(){
}

template<typename T>
bool ContactForceLimits<T>::Update_f(){}

template<typename T>
bool ContactForceLimits<T>::Update_A(){}

template<typename T>
bool ContactForceLimits<T>::Update_b(){}

template class ContactForceLimits<double>;
template class ContactForceLimits<float>;