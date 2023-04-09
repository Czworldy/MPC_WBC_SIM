#ifndef CONTACTFORCELIMITS_H
#define CONTACTFORCELIMITS_H

//Friction cone and lambda modulation

#include <WBC_CONTROL/wbc/task.h>
#include<WBC_CONTROL/dynamics/quadruped_dynamics_model.h>

template<typename T>
class ContactForceLimits: public Task<T>{
    public:
        ContactForceLimits(QuadrupedDynamicsModel* model);
        virtual ~ContactForceLimits();

        bool UpdateTask(const Eigen::Quaternion<T>& terrainOri);

        //DO NOT NEED
         bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des);

         bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des,
                                const Vec41<T>& contact_state);

         bool UpdateTask(const Vec31<T>* pos_des, 
                                const Vec31<T>* vel_des,
                                const Vec31<T>* acc_des,
                                const Vec41<T>& contact_state);

        bool UpdateTask(); 

        void TaskPrint();

    protected:
        virtual bool Update_A();
        virtual bool Update_b();
        virtual bool Update_D();
        virtual bool Update_f();

        virtual bool Update_size();

        Mat43<T> friction_cone_;
        UserParameter<T> user_p_;
        Mat3<T> terrainRotMat_;

        QuadrupedDynamicsModel*  _robot_sys;
};


#endif