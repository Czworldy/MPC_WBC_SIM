#ifndef EOMTASK_H
#define EOMTASK_H

//Floating base equations of motion
#include<WBC_CONTROL/wbc/task.h>
#include "WBC_CONTROL/dynamics/quadruped_dynamics_model.h"

template<typename T>
class EoMTask: public Task<T>{
    public:
        EoMTask(QuadrupedDynamicsModel* model);
        virtual ~EoMTask(){};

        virtual bool UpdateTask();
        //DO NOT NEED
        virtual bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des);

        virtual bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des,
                                const Vec41<T>& contact_state);

        virtual bool UpdateTask(const Vec31<T>* pos_des, 
                                const Vec31<T>* vel_des,
                                const Vec31<T>* acc_des,
                                const Vec41<T>& contact_state);

        virtual void TaskPrint();

    protected:
        virtual bool Update_A();
        virtual bool Update_b();
        virtual bool Update_D();
        virtual bool Update_f();

        virtual bool Update_size();
        
        QuadrupedDynamicsModel*  _robot_sys;
};
#endif