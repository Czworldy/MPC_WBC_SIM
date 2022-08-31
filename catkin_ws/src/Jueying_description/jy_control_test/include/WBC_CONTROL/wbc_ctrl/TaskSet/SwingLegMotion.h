#ifndef SWINGLEGMOTION_H
#define SWINGLEGMOTION_H

#include<task.h>
#include"quadruped_dynamics_model.h"

template <typename T>
class SwingLegMotion: public Task<T>{
    public:
        SwingLegMotion(QuadrupedDynamicsModel* model);
        virtual ~SwingLegMotion();

        virtual bool UpdateTask(const Vec31<T>* pos_des, 
                                const Vec31<T>* vel_des,
                                const Vec31<T>* acc_des,
                                const Vec41<T>& contact_state);

        //DO NOT NEED      
        virtual bool UpdateTask();
 
        virtual bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des);    

        virtual bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des,
                                const Vec41<T>& contact_state);

        virtual void TaskPrint();

    protected:
        virtual bool Update_A();
        virtual bool Update_b();
        virtual bool Update_D();
        virtual bool Update_f();

        virtual bool Update_size();
        
        QuadrupedDynamicsModel*  _robot_sys;
        Vec31<T> pos_d_[4];
        Vec31<T> vel_d_[4];
        Vec31<T> acc_d_[4];
        UserParameter<T> user_p_;
        Vec41<T> contactState;
        Vec31<T> Kp, Kd;
        Vec31<T> Kp_lf, Kd_lf;
        Vec31<T> Kp_rf, Kd_rf;
        Vec31<T> Kp_lh, Kd_lh;
        Vec31<T> Kp_rh, Kd_rh;
        int num_swing;
};


#endif