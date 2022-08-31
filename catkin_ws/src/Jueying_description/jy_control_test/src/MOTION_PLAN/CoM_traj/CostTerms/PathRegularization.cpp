#include "PathRegularization.h"
#include "AccelerationMin.h"
#include "time.h"

PathRegularization::PathRegularization(int nSegment, int iterationsBetweenSEG, double tk,UserParameter<double> param):
    //_nSegment(nSegment),
    _iterationsBetweenSEG(iterationsBetweenSEG),
    _tk(tk),
    _nSpline(nSegment){
        _W = param.weighPath;
        _Wdot = param.weighPath_dot;
        _Wddot = param.weighPath_ddot;

        n = 12*_nSpline;
        m = 6*(_nSpline+1);
        _cPATH = DVec<c_float>::Zero(n);
        solution_ = DVec<c_float>::Zero(n);

        //P
        P_x_ = new c_float[24*_nSpline];
        P_nnz_ = 24*_nSpline;
        P_i_ = new c_int[24*_nSpline];
        P_p_ = new c_int[n+1];
        //q
        q_ = new c_float[n];
        //A
        A_x_ = new c_float[36+60*(_nSpline-1)];
        A_nnz_ = 36+60*(_nSpline-1);
        A_i_ = new c_int[36+60*(_nSpline-1)];
        A_p_ = new c_int[n+1];
        //l
        l_ = new c_float[m];
        //u
        u_ = new c_float[m];

        // settings = new OSQPSettings;
        // data = new OSQPData;
}

PathRegularization::~PathRegularization(){
    delete [] P_x_;
    delete [] P_i_;
    delete [] P_p_;
    delete [] q_;
    delete [] A_x_;
    delete [] A_i_;
    delete [] A_p_;
    delete [] l_;
    delete [] u_;
    // delete settings;
    // delete data;
}

void PathRegularization::UpdateCostFunction(const Vec2<float>& initPoint_, const Vec2<float> &initVel_, const Vec2<float>& initAcc_,
                                            const Vec2<float>& finalPoint_, const Vec2<float> &finalVel_, const Vec2<float>& finalAcc_,
                                            c_float *P_PATH_x, c_float *q_PATH){

    initPoint = initPoint_;
    initVel = initVel_;
    initAcc = initAcc_;
    finalPoint = finalPoint_;
    finalVel = finalVel_;
    finalAcc = finalAcc_;
    
    _UpdateAccMin();
    _Update_EqConstraint();
    _SolveQP();
    _RecordData();
    _AbsDeviation(P_PATH_x, q_PATH);
}

void PathRegularization:: _UpdateAccMin(){
    AccelerationMin accelerationMin(_nSpline, _iterationsBetweenSEG, _tk);
    accelerationMin.UpdateCostFunction(P_x_, P_i_, P_p_);
    //q
    for(int i(0); i<n; i++)
        q_[i] = 0.;
}

void PathRegularization::_Update_EqConstraint(){

    //Spline_0
    c_float A_x_0[18] = {pow(_tk,5), 5*pow(_tk,4), 20*pow(_tk,3), 
                         pow(_tk,4), 4*pow(_tk, 3), 12*pow(_tk, 2),
                         pow(_tk,3), 3*pow(_tk,2), 6*_tk,
                         2, pow(_tk,2), 2*_tk, 2, 1, _tk, 1, 1, 1};
    c_int A_i_0[36] = {6,8,10,6,8,10,6,8,10,4,6,8,10,2,6,8,0,6,
                                         7,9,11,7,9,11,7,9,11,5,7,9,11,3,7,9,1,7};
    c_int A_p_0[12] = {0,3,6,9,13,16,18,21,24,27,31,34};

    for(int i(0); i<2;i++){
        for(int j(0); j<18; j++)
            A_x_[18*i+j] = A_x_0[j];
    }

    for(int i(0); i<36; i++)
        A_i_[i] = A_i_0[i];

    for(int i(0); i<12; i++)
        A_p_[i] = A_p_0[i]; 
    //printf("__________SPLINE_0__DONE________");
    //Spline_1 To End
    c_int A_i_kx[30] = {0, 2, 4, 6, 8, 10, 
                        0, 2, 4, 6, 8, 10, 
                        0, 2, 4, 6, 8, 10, 
                        0, 2, 4, 6, 8, 10,
                        0, 2,      6, 8,
                        0,           6};
    c_int A_i_ky[30] = {1, 3, 5, 7, 9, 11,
                        1, 3, 5, 7, 9, 11,
                        1, 3, 5, 7, 9, 11,
                        1, 3, 5, 7, 9, 11,
                        1,3,       7 ,9,
                        1,           7};
    c_int A_p_k[6] = {0, 6, 12, 18, 24, 28};

    for(int i(1); i<_nSpline;i++){
        double ta = _tk*i;
        double tb = ta + _tk;
        c_float A_x_k[30] = {-pow(ta,5), -5*pow(ta,4), -20*pow(ta, 3), pow(tb,5), 5*pow(tb,4), 20*pow(tb, 3),
                             -pow(ta,4), -4*pow(ta,3), -12*pow(ta,2), pow(tb,4),  4*pow(tb,3), 12*pow(tb,2),
                             -pow(ta,3), -3*pow(ta,2), -6*ta,                     pow(tb,3), 3*pow(tb,2), 6*tb,
                             -pow(ta,2), -2*ta, -2,                               pow(tb,2), 2*tb, 2,
                             -ta, -1,                                             tb, 1,
                             -1,                                                  1};
        for(int j(0); j<2; j++){
            for(int k(0); k<30; k++)
                A_x_ [36+60*(i-1)+30*j+k]= A_x_k[k];
        }

        for(int j(0); j<30; j++){
            A_i_[36+60*(i-1)+j] = A_i_kx[j]+6*i;
            A_i_[36+60*(i-1)+j+30] = A_i_ky[j]+6*i;
        }

        for(int j(0); j<6; j++){
            A_p_[12+12*(i-1)+j] = A_p_k[j] + 36 + 60*(i-1);
            A_p_[12+12*(i-1)+j+6] = A_p_k[j] + 36 + 60*(i-1)+30;
        }
    }
    A_p_[n] = 36+30*2*(_nSpline-1);

    //l,u
    for(int i(0); i<m; i++){
        l_[i] = 0.;
        u_[i] = 0.;
    }
    for(int i(0); i<2; i++){
        l_[i] = u_[i] = initPoint[i];
        l_[i+2] = u_[i+2] = initVel[i];
        l_[i+4] = u_[i+4] = initAcc[i];

        l_[m-6+i] =u_[m-6+i]= finalPoint[i];
        l_[m-4+i] =u_[m-4+i]= finalVel[i];
        l_[m-2+i] =u_[m-2+i]= finalAcc[i];
    }
    //printf("__________SPLINE_1__DONE________");
}

void PathRegularization::_SolveQP(){
    // Exitflag
    c_int exitflag = 0;

    // Workspace structures
    OSQPWorkspace *work;
    OSQPSettings  *settings = (OSQPSettings *)c_malloc(sizeof(OSQPSettings));
    OSQPData      *data     = (OSQPData *)c_malloc(sizeof(OSQPData));

    //Populate data
    if(data){
        data->n = n;
        data->m = m;
        data->P = csc_matrix(data->n, data->n, P_nnz_, P_x_, P_i_, P_p_);
        data->q = q_;
        data->A = csc_matrix(data->m, data->n, A_nnz_, A_x_, A_i_, A_p_);
        data->l = l_;
        data->u = u_;
    }
    
    //Define solver settings as default
    if (settings) {
        osqp_set_default_settings(settings);
    }

    // Setup workspace
    exitflag = osqp_setup(&work, data, settings);

    // Solve Problem
    osqp_solve(work);
    
    for(int i(0); i<n;i++){
        solution_[i] = (work->solution->x)[i];
    }

    // Cleanup
    osqp_cleanup(work);
    if (data) {
        if (data->A) c_free(data->A);
        if (data->P) c_free(data->P);
        c_free(data);
    }
    if (settings) c_free(settings);
}

void PathRegularization::_AbsDeviation(c_float *P_PATH_x_, c_float *q_PATH_){
    // c_int P_PATH_i_k[21] = {0,
    //                                                   0,1,
    //                                                   0,1,2,
    //                                                   0,1,2,3,
    //                                                   0,1,2,3,4,
    //                                                   0,1,2,3,4,5,};
    // c_int P_PATH_p_k[6] = {0,1,3,6,10,15,};

    double t;
    for(int i(0); i<_nSpline; i++){
        t = (i+1)*_tk;
        //position
        _T <<  pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1.;
        //velocity
        _Tdot << 5*pow(t, 4), 4*pow(t, 3), 3*pow(t, 2), 2*t, 1, 0.;
        //acceleration
        _Tddot << 20*pow(t, 3), 12*pow(t, 2), 6*t, 2., 0., 0.;

        Mat6<double> TT( _T.transpose()*_T);
        Mat6<double> TT_dot(_Tdot.transpose()*_Tdot);
        Mat6<double> TT_ddot(_Tddot.transpose()*_Tddot);

        int index = 0;
        for(int j(0); j<6; j++){
            for(int k(0); k<(j+1); k++){
                P_PATH_x_[42*i+index] = _W[0]*TT(k,j) +_Wdot[0]* TT_dot(k,j) +_Wddot[0]* TT_ddot(k,j);
                P_PATH_x_[42*i+index+21] = _W[1]*TT(k,j) +_Wdot[1]* TT_dot(k,j) +_Wddot[1]* TT_ddot(k,j);
                index++;
            }       
        }

        //P_PATH_i_
        // for(int j(0);j<21;j++){
        //     P_PATH_i_[42*i+j] = P_PATH_i_k[j]+12*i;
        //     P_PATH_i_[42*i+j+21] = P_PATH_i_k[j]+12*i+6;
        // }

        //P_PATH_p_
        // for(int j(0); j<6; j++){
        //     P_PATH_p_[12*i+j] = P_PATH_p_k[j]+42*i;
        //     P_PATH_p_[12*i+j+6] = P_PATH_p_k[j]+42*i+21;
        // }

        _cPATH.segment(12*i,6) =  - _W[0] * TT * solution_.segment(12*i,6) 
                                  - _Wdot[0] * TT_dot * solution_.segment(12*i,6);
                                  - _Wddot[0] * TT_ddot * solution_.segment(12*i,6);
        _cPATH.segment(12*i+6,6) =  -_W[1] * TT * solution_.segment(12*i+6,6);
                                    - _Wdot[1] * TT_dot * solution_.segment(12*i+6,6);
                                    - _Wddot[1] * TT_ddot * solution_.segment(12*i+6,6);
    }

    // P_PATH_p_[n] = 42*_nSpline;
    for(int i(0); i<n; i++)
        q_PATH_[i] = _cPATH[i];
    

}

void PathRegularization::_RecordData(){
    DMat<c_float> spline_xy(6*_nSpline, 2);
    for(int i(0); i<_nSpline; i++){
        spline_xy.block(6*i, 0, 6, 1) = solution_.segment(12*i, 6); 
        spline_xy.block(6*i, 1, 6, 1) = solution_.segment(12*i+6, 6); 
    }

    ofstream in;
    in.open("/home/MPC_WBC/dqwang/catkin_ws/src/Jueying_description/jy_control_test/Calculation_Analysis/Motion_Plan/ResultData/PathRegularizer.txt", ios::trunc);
    for(int i(0); i<6*_nSpline; i++)
        in << spline_xy(i,0) << "\t" << spline_xy(i,1) <<"\n";
    in.close();
}