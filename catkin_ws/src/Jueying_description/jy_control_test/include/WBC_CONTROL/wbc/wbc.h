#ifndef WBC_H
#define WBC_H

#include "Array.hh"
#include "QuadProg++.hh"
#include "task.h"
#include "cppTypes.h"
#include <vector>
#include "UserParameter.h"

#define WB WBC<T>

template <typename T>
class WBC{
    public:
        WBC(const UserParameter<T> & param);
         ~WBC(){}

        void UpdateSetting(const DMat<T>& M,
                                                 const DMat<T>& Js,
                                                 const DVec<T>& h,
                                                 const std::vector<Task<T>* >* task_list);
        void MakeTorque(DVec<T>& cmd);
    
    //protected:
        const std::vector<Task<T>* >* _task_list;

        void _Update_CostFuntion(int p1_);
        void _Update_InEqConstraint(int p1_); 
        void _Update_Nullspace(int p1_);
        void _SetOptimizationSize(int p1_);
        void _NullSpaceCal(const DMat<T>& A, DMat<T>& Anull );
        void pseudoInverse(DMat<T> const& matrix, double sigmaThreshold, DMat<T>& invMatrix);
        Eigen::VectorXd OsqpEigenSolve();


        // void _SetOptimizationSizeTEST(size_t p1_);
        // void _Update_NullspaceTEST(size_t p1_);
        // void _Update_CostFuntionTEST(size_t p1_);
        // void _Update_InEqConstraintTEST(size_t p1_); 
        // void MakeTorqueTEST(DVec<T>& cmd);

        //Var for QP____________________________________________
        quadprogpp::Vector<double> sol;
        //Cost
        quadprogpp::Matrix<double> G;
        quadprogpp::Vector<double> g0;
        //Equality
        quadprogpp::Matrix<double> CE;
        quadprogpp::Vector<double> ce0;
        //Inequality
        quadprogpp::Matrix<double> CI;
        quadprogpp::Vector<double> ci0;

        int dim_opt_; //dim_n_ + dim_nv_now
        int dim_n_; //dim_config_ + dim_contact_
        int dim_nv_now_, dim_nv_pre_;//(p+1)th dim_task_ineq
        int dim_Dbar_R_;//Rows of Dbar
        int dim_confi_; //dim_config_ of Task
        int dim_conta_;//dim_contact_ of Task
        int dim_eq_now_, dim_eq_pre_;//sum of dim_eq_
         
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        /*dim_opt_:58 dim_Dbar_R_:56 dim_n:30 dim_nv_now_:28*/
            //         H_  = DMat<T>::Zero(dim_opt_, dim_opt_);
            // c_ = DVec<T>::Zero(dim_opt_);
            // Dbar_ = DMat<T>::Zero(dim_Dbar_R_, dim_opt_);
            // fbar_ = DVec<T>::Zero(dim_Dbar_R_);
            // x_star_ = DVec<T>::Zero(dim_n_);
            // v_star_ = DVec<T>::Zero(dim_nv_now_);
            // N_stack_A_pre_ = DMat<T>::Identity(dim_n_, dim_n_); 
        Eigen::Matrix<T,-1,-1,0,58,58>  H_;
        Eigen::Matrix<T,-1,1,0,58,1>    c_;
        Eigen::Matrix<T,-1,-1,0,56,58>  Dbar_;
        Eigen::Matrix<T,-1,1,0,56,1>    fbar_;
        Eigen::Matrix<T,-1,1,0,30,1>    x_star_;
        Eigen::Matrix<T,-1,1,0,28,1>    v_star_;
        std::vector<Eigen::Matrix<T,-1,1,0,28,1>> _v_list;

        
        // DMat<T> H_;
        // DVec<T> c_;
        // DMat<T> Dbar_;
        // DVec<T> fbar_;
        // DVec<T> x_star_;//Optimal solution of last task
        // DVec<T> v_star_;
        // std::vector<DVec<T> > _v_list;
        DMat<T> stack_A_, stack_A_pre_;//A = [A1, A2, A3.....]
        DMat<T> N_stack_A_,N_stack_A_pre_;//NullSpace of A

        Eigen::Matrix<int, -1, 1, 0, 2, 1> Hier_; 
        Eigen::Matrix<T, -1, 1, 0, 8, 1> Q_; 
        int index_task_, index_task_pre_;

        //Var for UpdateSeting__________________________________________
        Eigen::Matrix<T,18,1> h_; //nonlinear effect_act
        DMat<T> M_Js_;//[M_act_; Js_act_]
    
        bool b_updatesetting_;
};

#endif
