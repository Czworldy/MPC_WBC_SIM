#ifndef CONTACTFORCELIMITS_H
#define CONTACTFORCELIMITS_H

//Friction cone and lambda modulation

#include <task.h>
#include<quadruped_dynamics_model.h>

template<typename T>
class ContactForceLimits: public Task<T>{
    public:
        ContactForceLimits(QuadrupedDynamicsModel* model);
        virtual ~ContactForceLimits();

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

        Mat43<T> friction_cone_;
        UserParameter<T> user_p_;

        QuadrupedDynamicsModel*  _robot_sys;
};


#endif