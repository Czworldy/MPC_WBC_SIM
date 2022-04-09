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

        void _Update_CostFuntion(size_t p1_);
        void _Update_InEqConstraint(size_t p1_); 
        void _Update_Nullspace(size_t p1_);
        void _SetOptimizationSize(size_t p1_);
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

        size_t dim_opt_; //dim_n_ + dim_nv_now
        size_t dim_n_; //dim_config_ + dim_contact_
        size_t dim_nv_now_, dim_nv_pre_;//(p+1)th dim_task_ineq
        size_t dim_Dbar_R_;//Rows of Dbar
        size_t dim_confi_; //dim_config_ of Task
        size_t dim_conta_;//dim_contact_ of Task
        size_t dim_eq_now_, dim_eq_pre_;//sum of dim_eq_
         
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        DMat<T> H_;
        DVec<T> c_;
        DMat<T> Dbar_;
        DVec<T> fbar_;
        DVec<T> x_star_;//Optimal solution of last task
        DVec<T> v_star_;
        std::vector<DVec<T> > _v_list;
        DMat<T> stack_A_, stack_A_pre_;//A = [A1, A2, A3.....]
        DMat<T> N_stack_A_,N_stack_A_pre_;//NullSpace of A

        Eigen::Matrix<int, -1, 1, 0, 2, 1> Hier_; 
        Eigen::Matrix<T, -1, 1, 0, 8, 1> Q_; 
        size_t index_task_, index_task_pre_;

        //Var for UpdateSeting__________________________________________
        DMat<T> h_; //nonlinear effect_act
        DMat<T> M_Js_;//[M_act_; Js_act_]
    
        bool b_updatesetting_;
};

#endif
