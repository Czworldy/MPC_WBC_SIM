#ifndef TASK_H
#define TASK_H

#include <cppTypes.h>
#include<PARAMETER/UserParameter.h>
// #include<ros/ros.h>
#define TK Task<T>

template<typename T>
class Task{
    public:
        Task():b_set_task_(false){}
        virtual ~Task(){}

        //Ax=b
        DMat<T> get_A(){return A_;}
        DVec<T> get_b(){return b_;}
        //Dx<=f
        DMat<T> get_D(){return D_;}
        DVec<T> get_f(){return f_;}

        bool IsTaskSet(){return b_set_task_;}
        size_t getDimOptVar(){return dim_optVar_;}
        size_t getDimTaskEq(){return dim_task_eq_;}
        size_t getDimTaskInEq(){return dim_task_ineq_;}
        size_t getDimConfig(){return dim_config_;}
        size_t getDimContact(){return dim_contact_;}
        void UnsetTask(){b_set_task_ = false;}

        virtual void TaskPrint() = 0;

        //EoMTask
        //ContactForceMin
        //NoContactMotion
        virtual bool UpdateTask() = 0; 
        //CoMAngularMotion
        //CoMLinearMotion
        virtual bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des){ return true; }
                                
        virtual bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des,
                                const Vec41<T>& contact_state){ return true; }
        //SwingLegMotion
        virtual bool UpdateTask(const Vec31<T>* pos_des, 
                                const Vec31<T>* vel_des,
                                const Vec31<T>* acc_des,
                                const Vec41<T>& contact_state){ return true; }

        // ContactForceLimits
        virtual bool UpdateTask(const Eigen::Quaternion<T>& terrainOri){
            printf("[Task] UpdateTask(const EigenQuat<T>& terrainOri) is not implemented\n");
            abort();
            return true;
        };

        virtual bool UpdateTask(const Vec12<T>& contactForce, const Vec41<T>& contactState){
            printf("[Task] UpdateTask(const Vec12<T>& contactForce, const Vec41<T>& contactState) is not implemented\n");
            abort();
            return true;
        }

    protected:

        virtual bool Update_A() = 0;
        virtual bool Update_b() = 0;
        virtual bool Update_D() = 0;
        virtual bool Update_f() = 0;

        virtual bool Update_size() = 0;

        bool b_set_task_;
        size_t dim_task_eq_;
        size_t dim_task_ineq_;
        size_t dim_optVar_;
        size_t dim_config_;
        size_t dim_contact_;

        DMat<T> A_;
        DVec<T> b_;
        DMat<T> D_;
        DVec<T> f_;
};
#endif