#ifndef COMANGULARMOTION_H
#define COMANGULARMOTION_H

#include<WBC_CONTROL/wbc/task.h>
#include<WBC_CONTROL/dynamics/quadruped_dynamics_model.h>
#include "Math/utility.h"
//TODO：未测试函数计算是否正确

template <typename T>
class CoMAngularMotion: public Task<T>{
    public:
        CoMAngularMotion(QuadrupedDynamicsModel* model);
        virtual ~CoMAngularMotion();

        virtual bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des,
                                const Vec41<T>& contact_state);
    
        virtual void TaskPrint();

        //DO NOT NEED
        virtual bool UpdateTask(); 
        virtual bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des);

        virtual bool UpdateTask(const Vec31<T>* pos_des, 
                                const Vec31<T>* vel_des,
                                const Vec31<T>* acc_des,
                                const Vec41<T>& contact_state);

    protected:
        virtual bool Update_A();
        virtual bool Update_b();
        virtual bool Update_D();
        virtual bool Update_f();

        virtual bool Update_size();

        QuadrupedDynamicsModel*  _robot_sys;
        DVec<T> omega_d_, acc_d_;
        Eigen::Quaternion<T> quat_d_;
        UserParameter<T> user_p_;
        Vec41<T> contactState;
        Vec31<T> Kp, Kd;

        Eigen::Quaternion<T> cur_quat_in_frame_c_inv_, des_quat_in_frame_c_;
        Vec31<T> omega_cur_in_frame_c_;
};
#endif