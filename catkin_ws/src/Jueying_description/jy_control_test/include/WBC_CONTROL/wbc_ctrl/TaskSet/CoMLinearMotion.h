#ifndef COMLINEARMOTION_H
#define COMLINEARMOTION_H

#include<task.h>
#include<quadruped_dynamics_model.h>

template <typename T>
class CoMLinearMotion: public Task<T>{
    public:
        CoMLinearMotion(QuadrupedDynamicsModel* model);
        virtual ~CoMLinearMotion();

        virtual bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des,
                                const Vec41<T>& contact_state);

        //DO NOT NEED
        virtual bool UpdateTask(); 

        virtual bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des);

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
        DVec<T> pos_d_, vel_d_, acc_d_;
        UserParameter<T> user_p_;
        Vec41<T> contactState;
        Vec31<T> Kp, Kd;
};

#endif