#pragma GCC optimize(2)
#include "wbc_ctrl.h"
#include "time.h"

template<typename T>
WBC_Ctrl<T>::WBC_Ctrl(QuadrupedDynamicsModel *model){
    _model = model;

    _eomTask = new EoMTask<T>(_model);
    _noContactMotion = new NoContactMotion<T>(_model);
    _contactForceLimits = new ContactForceLimits<T>(_model);
    _torqueLimits = new TorqueLimits<T>(_model);
    _comLinearMotion = new CoMLinearMotion<T>(_model);
    _comAngularMotion = new CoMAngularMotion<T>(_model);
    _swingLegMotion = new SwingLegMotion<T>(_model);
    _contactForceMin = new ContactForceMin<T>(_model);

    _wbc = new WBC<T>(param_);
}

template<typename T>
WBC_Ctrl<T>::~WBC_Ctrl(){
    
    typename std::vector<Task<T> *>::iterator iter = taskList.begin();
    while (iter < taskList.end()) {
    delete (*iter);
    ++iter;
  }
  taskList.clear();

  delete _wbc;
}

template<typename T>
void WBC_Ctrl<T>::_ComputeWBC(){
    _wbc->UpdateSetting(H_, CJ_, N_, &taskList);
    _wbc->MakeTorque(tau_);
}

template<typename T>
void WBC_Ctrl<T>::run(void* input, ControlFSMData<T>& data, DVec<T>&tau){

    _UpdateModel(data.bodyStateEst, data.legStateEst);
    _AllTaskUpdate(input);
    _ComputeWBC();
    tau = tau_;
}

template<typename T>
void WBC_Ctrl<T>::_UpdateModel(const BodyStateEstData<T> & bodyEst, 
                              const LegStateEstData<T> * legEst){//legEst为size=4的数组

    state_.contact_state_ = bodyEst.contactEstimate;//TODO: 接触状态是估计值还是规划值呢???
    state_.bodyOrientation.w() = bodyEst.base_orientation_world.w();
    state_.bodyOrientation.x() = bodyEst.base_orientation_world.x();
    state_.bodyOrientation.y() = bodyEst.base_orientation_world.y();
    state_.bodyOrientation.z() = bodyEst.base_orientation_world.z();
    state_.bodyRPY[0] = bodyEst.base_rpy_world[0];
    state_.bodyRPY[1] = bodyEst.base_rpy_world[1];
    state_.bodyRPY[2] = bodyEst.base_rpy_world[2];
    for(size_t i(0); i<3; i++){
        state_.bodyPosition[i] = bodyEst.base_pos_world[i];
        state_.bodyVelocity[i] = bodyEst.base_linear_vel_world[i];
        state_.bodyVelocity[i+3] = bodyEst.base_angular_vel_world[i];

        for(size_t leg(0); leg<4; ++leg){
            state_.q_leg[3*leg+i] = legEst[leg].q[i];
            state_.qd_leg[3*leg+i] = legEst[leg].qd[i];
        }
    }

    state_.frame_c_rpy_in_world[0] = bodyEst.frame_c_rpy_in_world[0];
    state_.frame_c_rpy_in_world[1] = bodyEst.frame_c_rpy_in_world[1];
    state_.frame_c_rpy_in_world[2] = bodyEst.frame_c_rpy_in_world[2];

    state_.frame_c_quat_in_world.w() = bodyEst.frame_c_quat_in_world.w();
    state_.frame_c_quat_in_world.x() = bodyEst.frame_c_quat_in_world.x();
    state_.frame_c_quat_in_world.y() = bodyEst.frame_c_quat_in_world.y();
    state_.frame_c_quat_in_world.z() = bodyEst.frame_c_quat_in_world.z();

    state_.frame_c_xyz_in_world[0] - bodyEst.frame_c_xyz_in_world[0];
    state_.frame_c_xyz_in_world[1] - bodyEst.frame_c_xyz_in_world[1];
    state_.frame_c_xyz_in_world[2] - bodyEst.frame_c_xyz_in_world[2];

    _model->setState(state_);

    _model->massMatrix();
    _model->contactJacobian();
    _model->nonlinearEffect();
    // _model->CoM6DJacobian();
    _model->CoM6DJacobian_c_frame();
    _model->transMatForTrackingTasks();

    H_ = _model->getMassMatrix().cast<T>();
    CJ_ = _model->getContactJacobian().cast<T>();
    N_ = _model->getNolinearEffect().cast<T>();

    // ROS_INFO_STREAM("MassMatrix: \n"<< H_);

}

template<typename T>
void WBC_Ctrl<T>::_AllTaskUpdate(void* input){
     _input_data = static_cast<LocomotionCtrlData<T>* >(input);

    _ClearUp();

    _eomTask->UpdateTask();
    _noContactMotion-> UpdateTask();
    _contactForceLimits-> UpdateTask();
    _torqueLimits-> UpdateTask();
    _comLinearMotion-> UpdateTask(_input_data->pBody_des,
                                  _input_data->vBody_des,
                                  _input_data->aBody_des,
                                  _input_data->contact_state);

    _comAngularMotion-> UpdateTask(_input_data->pBody_RPY_des,
                                   _input_data->vBody_RPY_des,
                                   _input_data->aBody_RPY_des,
                                   _input_data->contact_state);

    _swingLegMotion-> UpdateTask(_input_data->pFoot_des,
                                 _input_data->vFoot_des,
                                 _input_data->aFoot_des,
                                 _input_data->contact_state);

    _contactForceMin->UpdateTask();

    taskList.push_back(_eomTask);
    taskList.push_back(_torqueLimits);
    taskList.push_back(_contactForceLimits);
    taskList.push_back(_noContactMotion);
    taskList.push_back(_comLinearMotion);
    taskList.push_back(_comAngularMotion);
    taskList.push_back(_swingLegMotion);
    taskList.push_back(_contactForceMin);

}

template<typename T>
void WBC_Ctrl<T>::_ClearUp(){
    taskList.clear();
}

template class WBC_Ctrl<double>;
template class WBC_Ctrl<float>;