// #pragma GCC optimize(2)
#include "wbc.h"
#include "ros/ros.h"
#include <chrono>   
#include "OsqpEigen/OsqpEigen.h"
#include <omp.h>

using namespace std;
using namespace chrono;

template <typename T>
WBC<T>::WBC(const UserParameter<T> & param){

    Hier_ =param.hierWeig.hierarchy;
    Q_ = param.hierWeig.weight;


}

Eigen::VectorXd QPSolution_pre;
template <typename T>
Eigen::VectorXd WBC<T>::OsqpEigenSolve()
{
    //Var for Osqp
    Eigen::VectorXd QPSolution;
    Eigen::SparseMatrix<double> H_f(dim_opt_, dim_opt_), Dbar_f(dim_Dbar_R_, dim_opt_);
    Eigen::Matrix<double, -1, 1> c_f, fbar_f, lowerBound;
    c_f.resize(dim_opt_,1); fbar_f.resize(dim_Dbar_R_,1); 


    lowerBound = Eigen::VectorXd::Ones(dim_Dbar_R_);
    lowerBound *= - OsqpEigen::INFTY; 

    /*Set Osqp
    min    0.5x^THx + cx
    s.t.  lower<=Dx<=upper
    */
    omp_set_num_threads(4);

    for (int i = 0; i<dim_opt_;++i){
        for(int j = 0; j<dim_opt_;++j){
            if( i==j ) 
                H_f.insert(i,i) = H_(i,j) + 10e-6;   //??
            if(H_(i,j)!=0 && i!=j)
                H_f.insert(i,j) = H_(i,j);
        }
        c_f[i] = c_[i];
    }

    for (int i = 0; i<dim_Dbar_R_;++i){
        for(int j = 0; j<dim_opt_;++j){
            if(Dbar_(i,j) != 0)
                Dbar_f.insert(i,j) = Dbar_(i,j);
        }
        fbar_f[i] = fbar_[i];
    }
    
    OsqpEigen::Solver solver;


    // settings
    //solver.settings()->setVerbosity(false);
    // solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(dim_opt_);
    solver.data()->setNumberOfConstraints(dim_Dbar_R_);


    if(!solver.data()->setHessianMatrix(H_f)) 
        ROS_INFO("________OSQP_Failed_in_1_____________________________________");

    if(!solver.data()->setGradient(c_f)) 
        ROS_INFO("________OSQP_Failed_in_2_____________________________________");

    if(!solver.data()->setLinearConstraintsMatrix(Dbar_f)) 
        ROS_INFO("________OSQP_Failed_in_3_____________________________________");

    if(!solver.data()->setLowerBound(lowerBound)) 
        ROS_INFO("________OSQP_Failed_in_4_____________________________________");

    if(!solver.data()->setUpperBound(fbar_f)) 
        ROS_INFO("________OSQP_Failed_in_5_____________________________________");

    // instantiate the solver
    if(!solver.initSolver()) 
        ROS_INFO("________OSQP_Failed_in_6_____________________________________");

    if (!solver.solve()){
        ROS_INFO("________OSQP_Failed_in_7_____________________________________");
        QPSolution = QPSolution_pre;
    }
    else{    
        
        QPSolution = solver.getSolution();
        QPSolution_pre = QPSolution;
    }

    return QPSolution;

}

template<typename T>
void WBC<T>::MakeTorque(DVec<T>& cmd){
    if(!b_updatesetting_){
        ROS_INFO("[Warnning]WBC setting is not done\n");
    }

    //TASK1______________________________________________________________
    //dim
    Task<T>* task;
    dim_nv_now_ = 0;
    dim_eq_now_ = 0;
    for(int i(0); i<Hier_[0]; i++){
        task = (*_task_list)[i]; 
        dim_nv_now_+= task->getDimTaskInEq();
        dim_eq_now_+=task->getDimTaskEq();
    }
    dim_opt_ = dim_n_ + dim_nv_now_;
    dim_Dbar_R_ = 2*dim_nv_now_;

    //Resize H, c, Dbar, fbar
    /*dim_opt_:58 dim_Dbar_R_:56 dim_n:30 dim_nv_now_:28*/
    /*dim_opt_:30 dim_Dbar_R_:28 dim_n:30 dim_nv_now_:0*/

    H_  = DMat<T>::Zero(dim_opt_, dim_opt_);
    c_ = DVec<T>::Zero(dim_opt_);
    Dbar_ = DMat<T>::Zero(dim_Dbar_R_, dim_opt_);
    fbar_ = DVec<T>::Zero(dim_Dbar_R_);
    x_star_ = DVec<T>::Zero(dim_n_);
    v_star_ = DVec<T>::Zero(dim_nv_now_);
    N_stack_A_pre_ = DMat<T>::Identity(dim_n_, dim_n_); 

    //Set H, c, Dbar, fbar
    // DMat<T> H_LTCore, H_RBCore, Dbar_LTCore;
    // DVec<T> c_Head, fbar_Head;
    Eigen::Matrix<T, 30, 30> H_LTCore;
    Eigen::Matrix<T, 28, 28> H_RBCore;
    Eigen::Matrix<T, 30, 1>  c_Head;
    Eigen::Matrix<T, 28, 30>  Dbar_LTCore;
    Eigen::Matrix<T, 28, 1>  fbar_Head;

 
    int index_nv(0);
    H_LTCore = DMat<T>::Zero(dim_n_, dim_n_);
    H_RBCore = DMat<T>::Zero(dim_nv_now_, dim_nv_now_);
    c_Head = DVec<T>::Zero(dim_n_);
    Dbar_LTCore = DMat<T>::Zero(dim_nv_now_, dim_n_);
    fbar_Head = DVec<T>::Zero(dim_nv_now_);
    // std::cout<< "Hier_[0]" << Hier_[0] << "\n";
    for(int i(0); i<Hier_[0]; i++){
        Eigen::Matrix<T, -1, -1, 0, 12, 30>   A;
        Eigen::Matrix<T, -1, 1, 0, 12, 1>     b;
        Eigen::Matrix<T, -1, -1, 0, 24, 30>   D;
        Eigen::Matrix<T, -1, 1, 0, 24, 1>     f;
        int nv;
        // int eq;
        task = (*_task_list)[i];
        A = task->get_A();
        b = task->get_b();
        D = task->get_D();
        f = task->get_f();
        nv =  task->getDimTaskInEq();
        // eq = task->getDimTaskEq();
        //task->TaskPrint();
        H_LTCore += Q_[i]*A.transpose()*A;
        H_RBCore.block(index_nv, index_nv, nv, nv) = Q_[i]*DMat<T>::Identity(nv, nv);
        c_Head -= Q_[i]*A.transpose()*b;
        Dbar_LTCore.block(index_nv, 0, nv, dim_n_) = D;
        fbar_Head.segment(index_nv, nv) = f;

        index_nv+=nv;
    }

    H_.topLeftCorner(dim_n_, dim_n_) = H_LTCore;
    H_.bottomRightCorner(dim_nv_now_,dim_nv_now_) = H_RBCore;
    c_.head(dim_n_) = c_Head;

    Dbar_.topLeftCorner(dim_nv_now_, dim_n_) = Dbar_LTCore;
    Dbar_.topRightCorner(dim_nv_now_, dim_nv_now_) = - DMat<T>::Identity( dim_nv_now_,dim_nv_now_);
    Dbar_.bottomRightCorner(dim_nv_now_, dim_nv_now_) = - DMat<T>::Identity( dim_nv_now_,dim_nv_now_);
    fbar_.head(dim_nv_now_) = fbar_Head;

    //Set QP
    // Resize G, g0, CI, ci0
    // G.resize(dim_opt_, dim_opt_);
    // g0.resize(dim_opt_);
    // CI.resize(dim_opt_, dim_Dbar_R_);
    // ci0.resize(dim_Dbar_R_);
    // CE.resize(dim_opt_,0);
    // ce0.resize(0);
    // sol.resize(dim_opt_);

    // //Set
    // for (int i(0); i<dim_opt_;++i){
    //     for(int j(0); j<dim_opt_;++j){
    //         G[i][j] = H_(i,j);
    //         if(i==j)
    //             G[i][i]+=10e-5;//for float
    //     }
    //     g0[i] = c_[i];
    // }
    // for (int i(0); i<dim_Dbar_R_; ++i){
    //     for(int j(0); j<dim_opt_; ++j){
    //         CI[j][i] = -Dbar_(i,j);
    //     }
    //     ci0[i] = fbar_[i];
    // }
    // //Solve QP
    // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // double f = quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, sol);
    // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // std::cerr << "first time:" << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;

    //ROS_INFO("________TASK1_________Solve QP done!______________________________");
    //x*, v*
    // for(int i(0); i<dim_n_;++i)
    //     x_star_[i] = sol[i];
    // for(int i(0); i<dim_opt_ - dim_n_; ++i)
    //     v_star_[i] = sol[i+dim_n_];
    // _v_list.push_back(v_star_);

    // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    Eigen::VectorXd QP_rst = OsqpEigenSolve();
    // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // std::cerr << "first time:" << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;
    for(int i(0); i<dim_n_;++i)
        x_star_[i] = QP_rst[i];
    for(int i(0); i<dim_opt_ - dim_n_; ++i)
        v_star_[i] = QP_rst[i+dim_n_];
    _v_list.push_back(v_star_);

    //Task 2->end______________________________________________________________
    index_task_ = Hier_[0];
    index_task_pre_ = 0;
    dim_nv_pre_ = dim_nv_now_;
    dim_eq_pre_ = dim_eq_now_;
    for(int p(0); p<Hier_.rows()-1; p++){

        _Update_Nullspace(p+1);
        _SetOptimizationSize(p+1);
        _Update_CostFuntion(p+1);
        _Update_InEqConstraint(p+1);
        sol.resize(dim_opt_);

        // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        double f = quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, sol);
        // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        // std::cerr << "second time:" << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;

       // ROS_INFO("___________TASK2-end_______Solve QP done!______________________________");
        //x*, v*
        DVec<T> z_p1(dim_n_);
        for(int i(0); i<dim_n_; i++)
            z_p1[i] = sol[i];
        x_star_ =x_star_ +   N_stack_A_* z_p1;
        for(int i(0); i<dim_opt_ - dim_n_; ++i)
            v_star_[i] = sol[i+dim_n_];
        _v_list.push_back(v_star_);

        index_task_ += Hier_[p+1];
        index_task_pre_ += Hier_[p];
    }

    //Set cmd
    cmd.noalias() = M_Js_ * x_star_ + h_.bottomRows(dim_confi_ - 6);
}



template<typename T>
void WBC<T>::_Update_Nullspace(int p1_){
    
    if(dim_eq_pre_!=0){
        DMat<T> A_(dim_eq_pre_, dim_n_);
        int eq;
        int index_eq(0);
        for(int i(index_task_pre_); i<index_task_pre_+Hier_[p1_-1]; i++){
            // DMat<T> A;
            Eigen::Matrix<T, -1, -1, 0, 12, 30>   A;
            Task<T>* task;
            task = (*_task_list)[i];
            eq = task->getDimTaskEq();
            A = task->get_A();
            A_.block(index_eq, 0, eq, dim_n_) = A;
            index_eq+=eq;
        }
        DMat<T> N_;

        // auto start = system_clock::now();
        _NullSpaceCal(A_*N_stack_A_pre_, N_ );
        // auto end   = system_clock::now();
        // auto duration = duration_cast<microseconds>(end - start);
        // cout <<  "NULL SPACE Spent" << double(duration.count()) * microseconds::period::num / microseconds::period::den << " seconds." << endl;


        N_stack_A_ = N_stack_A_pre_ * N_;
        N_stack_A_pre_ = N_stack_A_ ;
    }
    //ROS_INFO("______________________Update_Nullspace_Done!______________________________");
}

template<typename T>
void WBC<T>::_SetOptimizationSize(int p1_){
    dim_nv_now_ = 0;
    dim_eq_now_ = 0;
    for(int i(index_task_); i<index_task_+Hier_[p1_];i++){
        Task<T>* task;
        task = (*_task_list)[i];
        dim_nv_now_+= task->getDimTaskInEq();
        dim_eq_now_+=task->getDimTaskEq();
        //task->TaskPrint();
        //ROS_INFO_STREAM("DimTaskInEq is "<<dim_nv_now_);
       // ROS_INFO("2_TO_END TASK");
    }
    dim_opt_ = dim_n_ + dim_nv_now_;
    dim_Dbar_R_ = 2*dim_nv_now_;
    for(int i(0); i< index_task_; i++){
        Task<T>* task;
        task = (*_task_list)[i];
        dim_Dbar_R_+=task->getDimTaskInEq();
    }

        /*dim_opt_:30 dim_Dbar_R_:28 dim_n:30 dim_nv_now_:0*/
    // printf("dim_opt_:%d dim_Dbar_R_:%d dim_n:%d dim_nv_now_:%d", dim_opt_,dim_Dbar_R_,dim_n_,dim_nv_now_);

    H_  = DMat<T>::Identity(dim_opt_, dim_opt_);
    c_ = DVec<T>::Zero(dim_opt_);
    Dbar_ = DMat<T>::Zero(dim_Dbar_R_, dim_opt_);
    fbar_ = DVec<T>::Zero(dim_Dbar_R_);
    v_star_ = DVec<T>::Zero(dim_nv_now_);

    G.resize(dim_opt_, dim_opt_);
    g0.resize(dim_opt_);
    CI.resize(dim_opt_, dim_Dbar_R_);
    ci0.resize(dim_Dbar_R_);
    CE.resize(dim_opt_,0);
    ce0.resize(0);
    sol.resize(dim_opt_);

    dim_nv_pre_ = dim_nv_now_;
    dim_eq_pre_ = dim_eq_now_;

    //ROS_INFO("______________________SetOptimizationSize_Done!______________________________");
}



template<typename T>
void WBC<T>::_Update_CostFuntion(int p1_){
    DMat<T> H_LTCore, H_RBCore;
    DVec<T> c_Head;
    int index_nv(0);
    H_LTCore = DMat<T>::Zero(dim_n_, dim_n_);
    H_RBCore = DMat<T>::Identity(dim_nv_now_, dim_nv_now_);
    c_Head = DVec<T>::Zero(dim_n_);

    for(int i(index_task_); i<index_task_+Hier_[p1_];i++){
        // DMat<T> A, b;
        Eigen::Matrix<T, -1, -1, 0, 12, 30>   A;
        Eigen::Matrix<T, -1, 1, 0, 12, 1>     b;
        int nv;
        Task<T>* task;
        task = (*_task_list)[i];
        A = task->get_A();
        b = task->get_b();
        nv = task->getDimTaskInEq();
        H_LTCore+=Q_[i] * N_stack_A_.transpose() * A.transpose() * A *  N_stack_A_;
        H_RBCore.block(index_nv, index_nv, nv, nv) = Q_[i]*DMat<T>::Identity(nv, nv);
        c_Head += Q_[i] * N_stack_A_.transpose()* A.transpose() *(A * x_star_ - b);

        index_nv+=nv;
    }

    H_.topLeftCorner(dim_n_, dim_n_) = H_LTCore;
    H_.bottomRightCorner(dim_nv_now_,dim_nv_now_) = H_RBCore;
    c_.head(dim_n_) = c_Head;

        //Set
    for (int i(0); i<dim_opt_;++i){
        for(int j(0); j<dim_opt_;++j){
            G[i][j] = H_(i,j);
            if(i==j)
                G[i][i]+=10e-5;//for float
        }
        g0[i] = c_[i];
    }
    //ROS_INFO("______________________Update_CostFuntion_Done!______________________________");
}


template<typename T>
void WBC<T>::_Update_InEqConstraint(int p1_){
    DMat<T> Dbar_LTCore, Dbar_LMCore;
    DVec<T> fbar_Head, fbar_Mid;
    int index_nv(0);
    Dbar_LTCore = DMat<T>::Zero(dim_nv_now_,dim_n_);
    Dbar_LMCore = DMat<T>::Zero(dim_Dbar_R_-2*dim_nv_now_, dim_n_);
    fbar_Head = DVec<T>::Zero(dim_nv_now_);
    fbar_Mid = DVec<T>::Zero(dim_Dbar_R_-2*dim_nv_now_);

    for(int i(index_task_); i<index_task_+Hier_[p1_];i++){
        Eigen::Matrix<T, -1, -1, 0, 24, 30>   D;
        Eigen::Matrix<T, -1, 1, 0, 24, 1>     f;
        int nv;
        Task<T>* task;
        task = (*_task_list)[i];
        D = task->get_D();
        f = task->get_f();       
        nv = task->getDimTaskInEq();
        Dbar_LTCore.block(index_nv, 0, nv, dim_n_) = D * N_stack_A_;
        fbar_Head.segment(index_nv, nv) = f - D * x_star_;

        index_nv +=nv;
    }

    index_nv=0;
    for(int i(0); i<index_task_; i++){
        Eigen::Matrix<T, -1, -1, 0, 24, 30>   D;
        Eigen::Matrix<T, -1, 1, 0, 24, 1>     f;
        int nv;
        Task<T>* task;
        task = (*_task_list)[i];
        D = task->get_D();
        f = task->get_f();
        nv = task->getDimTaskInEq();
        Dbar_LMCore.block(index_nv, 0, nv, dim_n_) =D * N_stack_A_;
        fbar_Mid.segment(index_nv, nv) = f - D*x_star_;   

        index_nv +=nv;
    }
    
    int index_Hier(0);
    for(int i(0); i<_v_list.size(); i++){
        fbar_Mid.segment(index_Hier , _v_list[i].rows()) += _v_list[i];  

        index_Hier += _v_list[i].rows();
    }

    Dbar_.topLeftCorner(dim_nv_now_, dim_n_) = Dbar_LTCore;
    Dbar_.topRightCorner(dim_nv_now_, dim_nv_now_) =  - DMat<T>::Identity( dim_nv_now_,dim_nv_now_);
    Dbar_.bottomRightCorner(dim_nv_now_, dim_nv_now_) = - DMat<T>::Identity( dim_nv_now_,dim_nv_now_);
    Dbar_.block(dim_nv_now_, 0, dim_Dbar_R_-2*dim_nv_now_, dim_n_) = Dbar_LMCore;

    fbar_.head(dim_nv_now_) = fbar_Head;
    fbar_.segment(dim_nv_now_,dim_Dbar_R_-2*dim_nv_now_) = fbar_Mid;

    for (int i(0); i<dim_Dbar_R_; ++i){
        for(int j(0); j<dim_opt_; ++j){
            CI[j][i] = -Dbar_(i,j);
        }
        ci0[i] = fbar_[i];
    }

    //ROS_INFO("______________________Update_InEqConstraint_Done!______________________________");

}



template <typename T>
void WBC<T>::UpdateSetting(const DMat<T>& M,
                           const DMat<T>& Js,
                           const DVec<T>& h,
                           const std::vector<Task<T>* >* task_list){
    _task_list = task_list;

    Task<T>* task;
    task = (*_task_list)[0];
    dim_n_ = task->getDimOptVar();
    dim_confi_ = task->getDimConfig();
    dim_conta_ = task->getDimContact();

    M_Js_ =DMat<T>::Zero(dim_confi_-6, dim_n_);
    M_Js_.leftCols(dim_confi_) = M.bottomRows(dim_confi_ - 6);
    M_Js_.rightCols(3*dim_conta_) = - Js.transpose().bottomRows(dim_confi_ - 6);
    h_ = h;
    b_updatesetting_ = true;
    _v_list.clear();
}

//svd    TODO: QR
// template<typename T>
// void WBC<T>::_NullSpaceCal(const DMat<T>& A, DMat<T>& Anull ){

// 	Eigen::JacobiSVD<DMat<T>> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
//     DMat<T> U = svd.matrixU();
// 	DMat<T> V = svd.matrixV();
// 	DMat<T> S = U.inverse() * A * V.transpose().inverse();
// 	for (int i=0; i < S.rows(); i++){
// 		if(S(i,i) < 0.5)
// 			S(i, i) = 1/(S(i, i) + 0.1);
// 		else
// 			S(i,i) = 1 / S(i, i);
// 	}
//     DMat<T> inv, ident;
//     int col;
//     col = A.cols();
//     ident = DMat<T>::Identity(col,col);
// 	inv = V * S.transpose() * U.transpose();
//     Anull = ident - inv * A;
// }
template<typename T>
void WBC<T>::_NullSpaceCal(const DMat<T>& A, DMat<T>& Anull ){

    DMat<T> inv, ident;
    int col;
    col = A.cols();
    ident = DMat<T>::Identity(col,col);
	this -> pseudoInverse(A, 0.0001, inv);
    Anull = ident - inv * A;
}

template <typename T>
void WBC<T>::pseudoInverse(DMat<T> const& matrix, double sigmaThreshold,
                           DMat<T>& invMatrix) {
  if (  (1 == matrix.rows()) && (1 == matrix.cols()) ) {
    invMatrix.resize(1, 1);
    if (matrix.coeff(0, 0) > sigmaThreshold) {
      invMatrix.coeffRef(0, 0) = 1.0 / matrix.coeff(0, 0);
    } else {
      invMatrix.coeffRef(0, 0) = 0.0;
    }
    return;
  }

  Eigen::JacobiSVD<DMat<T>> svd(matrix, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // not sure if we need to svd.sort()... probably not
  int const nrows(svd.singularValues().rows());
  DMat<T> invS;
  invS = DMat<T>::Zero(nrows, nrows);
  for (int ii(0); ii < nrows; ++ii) {
    if (svd.singularValues().coeff(ii) > sigmaThreshold) {
      invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
    } else {
      invS.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
      printf("sigular value is too small: %f\n",svd.singularValues().coeff(ii));
    }
  }
  invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
}

template class WBC<double>; 
template class WBC<float>; 
