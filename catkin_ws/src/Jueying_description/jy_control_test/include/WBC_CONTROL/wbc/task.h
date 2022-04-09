#ifndef TASK_H
#define TASK_H

#include <cppTypes.h>
#include<UserParameter.h>
#include<ros/ros.h>
#define TK Task<T>

template<typename T>
class Task{
    public:
        Task():b_set_task_(false){}
        virtual ~Task(){}

        // //Ax=b
        // DMat<T> get_A(){return A_;}
        // DVec<T> get_b(){return b_;}
        // //Dx<=f
        // DMat<T> get_D(){return D_;}
        // DVec<T> get_f(){return f_;}
        
        const Eigen::Matrix<T, -1, -1, 0, 12, 30>&  get_A(){return A_;}
        const Eigen::Matrix<T, -1, 1, 0, 12, 1>&  get_b(){return b_;}
        const Eigen::Matrix<T, -1, -1, 0, 24, 30>&  get_D(){return D_;}
        const Eigen::Matrix<T, -1, 1, 0, 24, 1>&  get_f(){return f_;}

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
                                const DVec<T>& acc_des) = 0;
                                
        virtual bool UpdateTask(const DVec<T>& pos_des, 
                                const DVec<T>& vel_des,
                                const DVec<T>& acc_des,
                                const Vec41<T>& contact_state) = 0;
        //SwingLegMotion
        virtual bool UpdateTask(const Vec31<T>* pos_des, 
                                const Vec31<T>* vel_des,
                                const Vec31<T>* acc_des,
                                const Vec41<T>& contact_state) = 0;

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

        // DMat<T> A_;
        // DVec<T> b_;
        // DMat<T> D_;
        // DVec<T> f_;

        Eigen::Matrix<T, -1, -1, 0, 12, 30>   A_;
        Eigen::Matrix<T, -1, 1, 0, 12, 1>     b_;
        Eigen::Matrix<T, -1, -1, 0, 24, 30>   D_;
        Eigen::Matrix<T, -1, 1, 0, 24, 1>     f_;
};
#endif