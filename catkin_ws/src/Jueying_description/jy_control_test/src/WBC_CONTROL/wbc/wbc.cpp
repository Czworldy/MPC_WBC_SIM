#pragma GCC optimize(2)
#include "wbc.h"
// #include "ros/ros.h"
#include <chrono>   
#include "OsqpEigen/OsqpEigen.h"

using namespace std;
using namespace chrono;

template <typename T>
WBC<T>::WBC(const UserParameter<T> & param){

    Hier_ =param.hierWeig.hierarchy;
    Q_ = param.hierWeig.weight;


}

template <typename T>
DVec<T> WBC<T>::OsqpEigenSolve()
{
    //Var for Osqp
    DVec<double> QPSolution;
    Eigen::SparseMatrix<double> H_f(dim_opt_, dim_opt_), Dbar_f(dim_Dbar_R_, dim_opt_);
    Eigen::Matrix<double, -1, 1> c_f, fbar_f, lowerBound;
    c_f.resize(dim_opt_,1); fbar_f.resize(dim_Dbar_R_,1); 


    lowerBound = DVec<double>::Ones(dim_Dbar_R_);
    lowerBound *= - OsqpEigen::INFTY; 

    /*Set Osqp
    min    0.5x^THx + c^Tx
    s.t.  lower<=Dx<=upper
    */

    for (uint16_t i(0); i<dim_opt_;++i){
        for(uint16_t j(0); j<dim_opt_;++j){
            // if( i==j ) 
            //     H_f.insert(i,i) = H_(i,j) + 10e-6;   //??
            // if(H_(i,j)!=0 && i!=j)
            //     H_f.insert(i,j) = H_(i,j);
            if(H_(i,j)!=0) H_f.insert(i,j) = H_(i,j);
        }
        c_f[i] = c_[i];
    }
    for (uint16_t i(0); i<dim_Dbar_R_;++i){
        for(uint16_t j(0); j<dim_opt_;++j){
            if(Dbar_(i,j) != 0)
                Dbar_f.insert(i,j) = Dbar_(i,j);
        }
        fbar_f[i] = fbar_[i];
    }
    
    OsqpEigen::Solver solver;


    solver.settings()->setAbsoluteTolerance(1);
    solver.settings()->setRelativeTolerance(1e-2);
    solver.settings()->setMaxIteration(20);
    // solver.settings()->setLinearSystemSolver(1);

    // solver.initSolver();

    // settings
    //solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(dim_opt_);
    solver.data()->setNumberOfConstraints(dim_Dbar_R_);


    if(!solver.data()->setHessianMatrix(H_f)) 
        printf("________OSQP_Failed_in_1_____________________________________");

    if(!solver.data()->setGradient(c_f)) 
        printf("________OSQP_Failed_in_2_____________________________________");

    if(!solver.data()->setLinearConstraintsMatrix(Dbar_f)) 
        printf("________OSQP_Failed_in_3_____________________________________");

    if(!solver.data()->setLowerBound(lowerBound)) 
        printf("________OSQP_Failed_in_4_____________________________________");

    if(!solver.data()->setUpperBound(fbar_f)) 
        printf("________OSQP_Failed_in_5_____________________________________");

    // instantiate the solver
    if(!solver.initSolver()) 
        printf("________OSQP_Failed_in_6_____________________________________");

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError){
        printf("________OSQP_Failed_in_7_____________________________________");
    }
    else{    
        QPSolution = solver.getSolution();
    }

    return QPSolution.cast<T>();

}

template<typename T>
void WBC<T>::MakeTorque(DVec<T>& cmd){
    if(!b_updatesetting_){
        printf("[Warnning]WBC setting is not done\n");
    }
    //eq constraint 0,3 emo ; no contact motion
    Task<T>* task;
    task = (*_task_list)[0];
    int dim_task_eq = (*_task_list)[0]->getDimTaskEq() + (*_task_list)[3]->getDimTaskEq();
    DMat<T> A_linearConstraint = DMat<T>::Zero(dim_task_eq, dim_n_);
    DVec<T> b_linearConstraint = DVec<T>::Zero(dim_task_eq);
    DMat<T> Aeq = task->get_A();
    A_linearConstraint.topRows(Aeq.rows()) = Aeq;
    b_linearConstraint.topRows(Aeq.rows()) = task->get_b();
    task = (*_task_list)[3];
    Aeq = task->get_A();
    A_linearConstraint.bottomRows(Aeq.rows()) = Aeq;
    b_linearConstraint.bottomRows(Aeq.rows()) = task->get_b();

    //ineq constraint 1,2 torque linmit; friction cone
    int dim_task_ineq = (*_task_list)[1]->getDimTaskInEq() + (*_task_list)[2]->getDimTaskInEq();
    DMat<T> D_ineqConstraint = DMat<T>::Zero(dim_task_ineq, dim_n_);
    DVec<T> f_ineqConstraint = DVec<T>::Zero(dim_task_ineq);
    task = (*_task_list)[1];
    D_ineqConstraint.topRows(task->getDimTaskInEq()) = task->get_D();
    f_ineqConstraint.topRows(task->getDimTaskInEq()) = task->get_f();
    task = (*_task_list)[2];
    D_ineqConstraint.bottomRows(task->getDimTaskInEq()) = task->get_D();
    f_ineqConstraint.bottomRows(task->getDimTaskInEq()) = task->get_f();

    //  cost function task 4,5,6,7 
    DMat<T> H_cost = DMat<T>::Zero(dim_n_, dim_n_);
    DVec<T> c_cost = DVec<T>::Zero(dim_n_);
    task = (*_task_list)[4];
    int dim_cost = (*_task_list)[4]->getDimTaskEq() + (*_task_list)[5]->getDimTaskEq() + (*_task_list)[6]->getDimTaskEq() + (*_task_list)[7]->getDimTaskEq();
    DMat<T> A_cost = DMat<T>::Zero(dim_cost, dim_n_);
    DVec<T> b_cost = DVec<T>::Zero(dim_cost);
    A_cost.topRows(task->getDimTaskEq()) = task->get_A()*10;
    b_cost.topRows(task->getDimTaskEq()) = task->get_b()*10;
    task = (*_task_list)[5];
    A_cost.middleRows((*_task_list)[4]->getDimTaskEq(), task->getDimTaskEq()) = task->get_A()*5;
    b_cost.middleRows((*_task_list)[4]->getDimTaskEq(), task->getDimTaskEq()) = task->get_b()*5;
    task = (*_task_list)[6];
    A_cost.middleRows((*_task_list)[4]->getDimTaskEq() + (*_task_list)[5]->getDimTaskEq(), task->getDimTaskEq()) = task->get_A();
    b_cost.middleRows((*_task_list)[4]->getDimTaskEq() + (*_task_list)[5]->getDimTaskEq(), task->getDimTaskEq()) = task->get_b();
    task = (*_task_list)[7];
    A_cost.bottomRows(task->getDimTaskEq()) = task->get_A()*0.01;
    b_cost.bottomRows(task->getDimTaskEq()) = task->get_b()*0.01;

    H_cost = A_cost.transpose()*A_cost;
    c_cost = -A_cost.transpose()*b_cost;

        //Var for Osqp
        // std::cout << "osqp\n";
    DVec<double> QPSolution;
    DMat<T> A_D(dim_task_ineq+dim_task_eq, dim_n_);
    DVec<T> b_f(dim_task_ineq+dim_task_eq);
    A_D.topRows(dim_task_eq) = A_linearConstraint;
    A_D.bottomRows(dim_task_ineq) = D_ineqConstraint;
    b_f.topRows(dim_task_eq) = b_linearConstraint;
    b_f.bottomRows(dim_task_ineq) = f_ineqConstraint;
        // std::cout << "osqp1\n";

    Eigen::SparseMatrix<double> H_sparse(dim_n_, dim_n_), A_D_sparse(dim_task_ineq+dim_task_eq, dim_n_);
    DVec<double> lowerBound(dim_task_ineq+dim_task_eq), upperBound(dim_task_ineq+dim_task_eq), gradient(dim_n_);

    for (int i = 0; i < dim_task_eq; i++)
    {
        lowerBound(i) = b_linearConstraint(i);
    }
    
    // lowerBound.topRows(dim_task_eq) = b_linearConstraint.cast<double>();
    lowerBound.tail(dim_task_ineq).setConstant(-1e30);

    /*Set Osqp
    min    0.5x^THx + c^Tx
    s.t.  lower<=Dx<=upper
    */
        // std::cout << "osqp2\n";

    for (uint16_t i(0); i<dim_n_;++i){
        for(uint16_t j(0); j<dim_n_;++j){
            // if( i==j ) 
            //     H_f.insert(i,i) = H_(i,j) + 10e-6;   //??
            // if(H_(i,j)!=0 && i!=j)
            //     H_f.insert(i,j) = H_(i,j);
            if(H_cost(i,j)!=0) H_sparse.insert(i,j) = H_cost(i,j);
        }
        gradient(i) = c_cost(i);
    }
        // std::cout << "osqp3\n";

    for (uint16_t i(0); i<dim_task_ineq+dim_task_eq;++i){
        for(uint16_t j(0); j<dim_n_;++j){
            if(A_D(i,j) != 0)
                A_D_sparse.insert(i,j) = A_D(i,j);
        }
        upperBound(i) = b_f(i);
    }
    
    OsqpEigen::Solver solver;


    // solver.settings()->setAbsoluteTolerance(1);
    // solver.settings()->setRelativeTolerance(1e-2);
    // solver.settings()->setMaxIteration(20);
    // solver.settings()->setLinearSystemSolver(1);

    // solver.initSolver();

    // settings
    //solver.settings()->setVerbosity(false);
    // solver.settings()->setWarmStart(true);

    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(dim_n_);
    solver.data()->setNumberOfConstraints(dim_task_ineq+dim_task_eq);


    if(!solver.data()->setHessianMatrix(H_sparse)) 
        printf("________OSQP_Failed_in_1_____________________________________");

    if(!solver.data()->setGradient(gradient)) 
        printf("________OSQP_Failed_in_2_____________________________________");

    if(!solver.data()->setLinearConstraintsMatrix(A_D_sparse)) 
        printf("________OSQP_Failed_in_3_____________________________________");

    if(!solver.data()->setLowerBound(lowerBound)) 
        printf("________OSQP_Failed_in_4_____________________________________");

    if(!solver.data()->setUpperBound(upperBound)) 
        printf("________OSQP_Failed_in_5_____________________________________");

    // instantiate the solver
    if(!solver.initSolver()) 
        printf("________OSQP_Failed_in_6_____________________________________");

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError){
        printf("________OSQP_Failed_in_7_____________________________________");
    }
    else{    
        QPSolution = solver.getSolution();
    }
    // std::cout << "QPSolution : " << QPSolution.transpose() << std::endl;

    DVec<T> toque_ = M_Js_ * QPSolution.cast<T>() + h_.bottomRows(dim_confi_ - 6);
    // std::cout << "QPSolution : " << QPSolution.transpose() << std::endl;

    // std::cout << "toque_ : " << toque_.transpose() << std::endl;


    //TASK1______________________________________________________________
    //dim
    // Task<T>* task;
    // dim_nv_now_ = 0;
    // dim_eq_now_ = 0;
    // for(size_t i(0); i<Hier_[0]; i++){
    //     task = (*_task_list)[i]; 
    //     dim_nv_now_+= task->getDimTaskInEq();
    //     dim_eq_now_+=task->getDimTaskEq();
    // }
    // dim_opt_ = dim_n_ + dim_nv_now_;
    // dim_Dbar_R_ = 2*dim_nv_now_;

    // //Resize H, c, Dbar, fbar
    // H_  = DMat<T>::Zero(dim_opt_, dim_opt_);
    // c_ = DVec<T>::Zero(dim_opt_);
    // Dbar_ = DMat<T>::Zero(dim_Dbar_R_, dim_opt_);
    // fbar_ = DVec<T>::Zero(dim_Dbar_R_);
    // x_star_ = DVec<T>::Zero(dim_n_);
    // v_star_ = DVec<T>::Zero(dim_nv_now_);
    // N_stack_A_pre_ = DMat<T>::Identity(dim_n_, dim_n_); 

    // //Set H, c, Dbar, fbar
    // DMat<T> H_LTCore, H_RBCore, Dbar_LTCore;
    // DVec<T> c_Head, fbar_Head;
    // size_t index_nv(0);
    // H_LTCore = DMat<T>::Zero(dim_n_, dim_n_);
    // H_RBCore = DMat<T>::Zero(dim_nv_now_, dim_nv_now_);
    // c_Head = DVec<T>::Zero(dim_n_);
    // Dbar_LTCore = DMat<T>::Zero(dim_nv_now_, dim_n_);
    // fbar_Head = DVec<T>::Zero(dim_nv_now_);
    // // std::cout<< "Hier_[0]" << Hier_[0] << "\n";
    // for(size_t i(0); i<Hier_[0]; i++){
    //     DMat<T> A, b, D,f;
    //     size_t nv;
    //     size_t eq;
    //     task = (*_task_list)[i];
    //     A = task->get_A();
    //     b = task->get_b();
    //     D = task->get_D();
    //     f = task->get_f();
    //     nv =  task->getDimTaskInEq();
    //     eq = task->getDimTaskEq();
    //     //task->TaskPrint();
    //     H_LTCore += Q_[i]*A.transpose()*A;
    //     H_RBCore.block(index_nv, index_nv, nv, nv) = Q_[i]*DMat<T>::Identity(nv, nv);
    //     c_Head -= Q_[i]*A.transpose()*b;
    //     Dbar_LTCore.block(index_nv, 0, nv, dim_n_) = D;
    //     fbar_Head.segment(index_nv, nv) = f;

    //     index_nv+=nv;
    // }

    // H_.topLeftCorner(dim_n_, dim_n_) = H_LTCore;
    // H_.bottomRightCorner(dim_nv_now_,dim_nv_now_) = H_RBCore;
    // c_.head(dim_n_) = c_Head;

    // Dbar_.topLeftCorner(dim_nv_now_, dim_n_) = Dbar_LTCore;
    // Dbar_.topRightCorner(dim_nv_now_, dim_nv_now_) = - DMat<T>::Identity( dim_nv_now_,dim_nv_now_);
    // Dbar_.bottomRightCorner(dim_nv_now_, dim_nv_now_) = - DMat<T>::Identity( dim_nv_now_,dim_nv_now_);
    // fbar_.head(dim_nv_now_) = fbar_Head;

    // //Set QP
    // // Resize G, g0, CI, ci0
    // // G.resize(dim_opt_, dim_opt_);
    // // g0.resize(dim_opt_);
    // // CI.resize(dim_opt_, dim_Dbar_R_);
    // // ci0.resize(dim_Dbar_R_);
    // // CE.resize(dim_opt_,0);
    // // ce0.resize(0);
    // // sol.resize(dim_opt_);

    // // //Set
    // // for (size_t i(0); i<dim_opt_;++i){
    // //     for(size_t j(0); j<dim_opt_;++j){
    // //         G[i][j] = H_(i,j);
    // //         if(i==j)
    // //             G[i][i]+=10e-5;//for float
    // //     }
    // //     g0[i] = c_[i];
    // // }
    // // for (size_t i(0); i<dim_Dbar_R_; ++i){
    // //     for(size_t j(0); j<dim_opt_; ++j){
    // //         CI[j][i] = -Dbar_(i,j);
    // //     }
    // //     ci0[i] = fbar_[i];
    // // }
    // // //Solve QP
    // // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // // double f = quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, sol);
    // // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // // std::cerr << "first time:" << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;

    // //printf("________TASK1_________Solve QP done!______________________________");
    // //x*, v*
    // // for(size_t i(0); i<dim_n_;++i)
    // //     x_star_[i] = sol[i];
    // // for(size_t i(0); i<dim_opt_ - dim_n_; ++i)
    // //     v_star_[i] = sol[i+dim_n_];
    // // _v_list.push_back(v_star_);

    // std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    // DVec<T> QP_rst = OsqpEigenSolve();
    // std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    // std::cerr << "first time:" << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;
    
    // for(size_t i(0); i<dim_n_;++i)
    //     x_star_[i] = QP_rst[i];
    // for(size_t i(0); i<dim_opt_ - dim_n_; ++i)
    //     v_star_[i] = QP_rst[i+dim_n_];
    // _v_list.push_back(v_star_);

    // //Task 2->end______________________________________________________________
    // index_task_ = Hier_[0];
    // index_task_pre_ = 0;
    // dim_nv_pre_ = dim_nv_now_;
    // dim_eq_pre_ = dim_eq_now_;
    // for(size_t p(0); p<Hier_.rows()-1; p++){

    //     _Update_Nullspace(p+1);
    //     _SetOptimizationSize(p+1);
    //     _Update_CostFuntion(p+1);
    //     _Update_InEqConstraint(p+1);
    //     sol.resize(dim_opt_);

    //     std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    //     double f = quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, sol);
    //     std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    //     std::cerr << "second time:" << std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count() << "us" << std::endl;

    //    // printf("___________TASK2-end_______Solve QP done!______________________________");
    //     //x*, v*
    //     DVec<T> z_p1(dim_n_);
    //     for(size_t i(0); i<dim_n_; i++)
    //         z_p1[i] = sol[i];
    //     x_star_ =x_star_ +   N_stack_A_* z_p1;
    //     for(size_t i(0); i<dim_opt_ - dim_n_; ++i)
    //         v_star_[i] = sol[i+dim_n_];
    //     _v_list.push_back(v_star_);

    //     index_task_ += Hier_[p+1];
    //     index_task_pre_ += Hier_[p];
    // }

    //Set cmd
    // cmd = M_Js_ * x_star_ + h_.bottomRows(dim_confi_ - 6);
    cmd = toque_;
}



template<typename T>
void WBC<T>::_Update_Nullspace(size_t p1_){
    
    if(dim_eq_pre_!=0){
        DMat<T> A_(dim_eq_pre_, dim_n_);
        size_t eq;
        size_t index_eq(0);
        for(size_t i(index_task_pre_); i<index_task_pre_+Hier_[p1_-1]; i++){
            DMat<T> A;
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
    //printf("______________________Update_Nullspace_Done!______________________________");
}

template<typename T>
void WBC<T>::_SetOptimizationSize(size_t p1_){
    dim_nv_now_ = 0;
    dim_eq_now_ = 0;
    for(size_t i(index_task_); i<index_task_+Hier_[p1_];i++){
        Task<T>* task;
        task = (*_task_list)[i];
        dim_nv_now_+= task->getDimTaskInEq();
        dim_eq_now_+=task->getDimTaskEq();
        //task->TaskPrint();
        //printf_STREAM("DimTaskInEq is "<<dim_nv_now_);
       // printf("2_TO_END TASK");
    }
    dim_opt_ = dim_n_ + dim_nv_now_;
    dim_Dbar_R_ = 2*dim_nv_now_;
    for(size_t i(0); i< index_task_; i++){
        Task<T>* task;
        task = (*_task_list)[i];
        dim_Dbar_R_+=task->getDimTaskInEq();
    }

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

    //printf("______________________SetOptimizationSize_Done!______________________________");
}



template<typename T>
void WBC<T>::_Update_CostFuntion(size_t p1_){
    DMat<T> H_LTCore, H_RBCore;
    DVec<T> c_Head;
    size_t index_nv(0);
    H_LTCore = DMat<T>::Zero(dim_n_, dim_n_);
    H_RBCore = DMat<T>::Identity(dim_nv_now_, dim_nv_now_);
    c_Head = DVec<T>::Zero(dim_n_);

    for(size_t i(index_task_); i<index_task_+Hier_[p1_];i++){
        DMat<T> A, b;
        size_t nv;
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
    for (size_t i(0); i<dim_opt_;++i){
        for(size_t j(0); j<dim_opt_;++j){
            G[i][j] = H_(i,j);
            if(i==j)
                G[i][i]+=10e-5;//for float
        }
        g0[i] = c_[i];
    }
    //printf("______________________Update_CostFuntion_Done!______________________________");
}


template<typename T>
void WBC<T>::_Update_InEqConstraint(size_t p1_){
    DMat<T> Dbar_LTCore, Dbar_LMCore;
    DVec<T> fbar_Head, fbar_Mid;
    size_t index_nv(0);
    Dbar_LTCore = DMat<T>::Zero(dim_nv_now_,dim_n_);
    Dbar_LMCore = DMat<T>::Zero(dim_Dbar_R_-2*dim_nv_now_, dim_n_);
    fbar_Head = DVec<T>::Zero(dim_nv_now_);
    fbar_Mid = DVec<T>::Zero(dim_Dbar_R_-2*dim_nv_now_);

    for(size_t i(index_task_); i<index_task_+Hier_[p1_];i++){
        DMat<T> D;
        DVec<T> f;
        size_t nv;
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
    for(size_t i(0); i<index_task_; i++){
        DMat<T> D;
        DVec<T> f;
        size_t nv;
        Task<T>* task;
        task = (*_task_list)[i];
        D = task->get_D();
        f = task->get_f();
        nv = task->getDimTaskInEq();
        Dbar_LMCore.block(index_nv, 0, nv, dim_n_) =D * N_stack_A_;
        fbar_Mid.segment(index_nv, nv) = f - D*x_star_;   

        index_nv +=nv;
    }
    
    size_t index_Hier(0);
    for(size_t i(0); i<_v_list.size(); i++){
        fbar_Mid.segment(index_Hier , _v_list[i].rows()) += _v_list[i];  

        index_Hier += _v_list[i].rows();
    }

    Dbar_.topLeftCorner(dim_nv_now_, dim_n_) = Dbar_LTCore;
    Dbar_.topRightCorner(dim_nv_now_, dim_nv_now_) =  - DMat<T>::Identity( dim_nv_now_,dim_nv_now_);
    Dbar_.bottomRightCorner(dim_nv_now_, dim_nv_now_) = - DMat<T>::Identity( dim_nv_now_,dim_nv_now_);
    Dbar_.block(dim_nv_now_, 0, dim_Dbar_R_-2*dim_nv_now_, dim_n_) = Dbar_LMCore;

    fbar_.head(dim_nv_now_) = fbar_Head;
    fbar_.segment(dim_nv_now_,dim_Dbar_R_-2*dim_nv_now_) = fbar_Mid;

    for (size_t i(0); i<dim_Dbar_R_; ++i){
        for(size_t j(0); j<dim_opt_; ++j){
            CI[j][i] = -Dbar_(i,j);
        }
        ci0[i] = fbar_[i];
    }

    //printf("______________________Update_InEqConstraint_Done!______________________________");

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
//     size_t col;
//     col = A.cols();
//     ident = DMat<T>::Identity(col,col);
// 	inv = V * S.transpose() * U.transpose();
//     Anull = ident - inv * A;
// }
template<typename T>
void WBC<T>::_NullSpaceCal(const DMat<T>& A, DMat<T>& Anull ){

    DMat<T> inv, ident;
    size_t col;
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

// template class WBC<double>; 
template class WBC<float>; 
