#ifndef  WBC_CONTROLLER_H
#define WBC_CONTROLLER_H

#include "quadruped_dynamics_model.h"
#include "wbc.h"
#include <cppTypes.h>
#include "ControlFSMData.h"
#include "UserParameter.h"

#include "task.h"
#include "EoMTask.h"
#include "NoContactMotion.h"
#include "ContactForceLimits.h"
#include "TorqueLimits.h"
#include "CoMLinearMotion.h"
#include "CoMAngularMotion.h"
#include "SwingLegMotion.h"
#include "ContactForceMin.h"

#define WBCtrl WBC_Ctrl<T>

template<typename T>
class LocomotionCtrlData{
    public:
        Vec31<T> pBody_des;
        Vec31<T> vBody_des;
        Vec31<T> aBody_des;
        Vec31<T> pBody_RPY_des;
        Vec31<T> vBody_RPY_des;
        Vec31<T> aBody_RPY_des;

        Vec31<T> pFoot_des[4];
        Vec31<T> vFoot_des[4];
        Vec31<T> aFoot_des[4];

        Vec41<T> contact_state; 
};

template<typename T>
class WBC_Ctrl{
    public:
        WBC_Ctrl(QuadrupedDynamicsModel *model);
        ~WBC_Ctrl();

        void run(void* input, ControlFSMData<T>& data, DVec<T>&tau);
    
    protected:
        void _AllTaskUpdate(void* input);
        void _UpdateModel(const BodyStateEstData<T> & bodyEst, 
                                                  const LegStateEstData<T> * legEst);
        void _ComputeWBC();

        void _ClearUp();

        WBC<T>* _wbc;
        QuadrupedDynamicsModel* _model;
        std::vector<Task<T>*> taskList;

        DMat<T> H_;
        DMat<T> CJ_;
        DVec<T> N_;

        DVec<T> tau_;

        FBModelState<double> state_;
        UserParameter<T> param_;

        LocomotionCtrlData<T>* _input_data;

        Task<T>* _eomTask;
        Task<T>* _noContactMotion;
        Task<T>* _contactForceLimits;
        Task<T>* _torqueLimits;
        Task<T>* _comLinearMotion;
        Task<T>* _comAngularMotion;
        Task<T>* _swingLegMotion;
        Task<T>* _contactForceMin;
};
#endif
