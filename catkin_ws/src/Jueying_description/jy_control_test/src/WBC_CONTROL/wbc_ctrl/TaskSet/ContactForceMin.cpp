#include "WBC_CONTROL/wbc_ctrl/TaskSet/ContactForceMin.h"

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
bool ContactForceMin<T>::UpdateTask(const Vec12<T>& contactForce, const Vec41<T>& contactState){
    contactState_ = contactState;
    Update_size();
    Update_A();
    // std::cout << "contactForce: " << contactForce.transpose() << std::endl;
    // std::cout << "contactState: " << contactState_.transpose() << std::endl;
    Update_b(contactForce); //Feet Contact Forces: [LF, RF, LH, RH]
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
    // std::cout << "A_ = \n" << TK::A_ << std::endl;

    return true;
}

template <typename T>
bool ContactForceMin<T>::Update_b(const Vec12<T>& contactForce){
    // TK::b_.setZero();
    std::vector<Vec31<T>> contactForceVec;
    
    if(contactState_[legID::LF]){
        contactForceVec.push_back(contactForce.segment(0,3));
    }
    if(contactState_[legID::LB]){
        contactForceVec.push_back(contactForce.segment(6,3));
    }
    if(contactState_[legID::RF]){
        contactForceVec.push_back(contactForce.segment(3,3));
    }
    if(contactState_[legID::RB]){
        contactForceVec.push_back(contactForce.segment(9,3));
    }


    for (int i = 0; i < contactForceVec.size(); i++){
        TK::b_.segment(3*i, 3) = contactForceVec[i];
    }

    // std::cout << "b_ = \n" << TK::b_.transpose() << std::endl;
    
    
    return true;
}

template <typename T>
void ContactForceMin<T>::TaskPrint(){
    printf("TASK_PRINT_CONTACTFORCEMIN");
}


template<typename T>
bool ContactForceMin<T>::UpdateTask(const DVec<T>& pos_des, 
                                    const DVec<T>& vel_des,
                                    const DVec<T>& acc_des){
    printf("CONTACTFORCEMIN ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");return true;
}

template<typename T>
bool ContactForceMin<T>::UpdateTask(const DVec<T>& pos_des, 
                                    const DVec<T>& vel_des,
                                    const DVec<T>& acc_des,
                                    const Vec41<T>& contact_state){
printf("CONTACTFORCEMIN ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");  return true;                                 
}

template<typename T>
bool ContactForceMin<T>::UpdateTask(const Vec31<T>* pos_des, 
                                    const Vec31<T>* vel_des,
                                    const Vec31<T>* acc_des,
                                    const Vec41<T>& contact_state){
printf("CONTACTFORCEMIN ERROR: YOU LOAD WRONG UPDATE TASK FUNCTION!");   return true;                                
}


template<typename T>
bool ContactForceMin<T>::Update_b(){return true; }

template<typename T>
bool ContactForceMin<T>::Update_D(){return true; }

template<typename T>
bool ContactForceMin<T>::Update_f(){ return true;}

template class ContactForceMin<double>;
template class ContactForceMin<float>;