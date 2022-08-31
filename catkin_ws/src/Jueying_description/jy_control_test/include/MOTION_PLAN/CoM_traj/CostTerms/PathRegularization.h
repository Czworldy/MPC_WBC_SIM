#ifndef PATHREGULARIZATION_H
#define PATHREGULARIZATION_H

#include "cppTypes.h"
#include "UserParameter.h"
#include "osqp.h"

#include <fstream>

class PathRegularization{
    public:
        PathRegularization(int nSegment, int iterationsBetweenSEG, double tk, UserParameter<double> param);
        ~PathRegularization();

        void UpdateCostFunction(const Vec2<float>& initPoint_, const Vec2<float> &initVel_, const Vec2<float>& initAcc_,
                                const Vec2<float>& finalPoint_, const Vec2<float> &finalVel_, const Vec2<float>& finalAcc_,
                                c_float *P_PATH_x, c_float *q_PATH);
    
    protected:
        void _AbsDeviation(c_float *P_PATH_x_, c_float *q_PATH_);
        void _SolveQP();
        void _UpdateAccMin();
        void _Update_EqConstraint();

        void _RecordData();

        DVec<double> _W;
        DVec<double> _Wdot;
        DVec<double> _Wddot;
       
        //Var for QP
        //P
        c_float* P_x_;
        c_int P_nnz_;
        c_int* P_i_;
        c_int* P_p_;
        //q
        c_float* q_;
        //A
        c_float* A_x_;
        c_int A_nnz_;
        c_int* A_i_;
        c_int* A_p_;
        //l
        c_float* l_;
        //u
        c_float* u_;
        //n,m
        c_int n;
        c_int m;

        // OSQPWorkspace *work;
        // OSQPSettings *settings;
        // OSQPData *data;

        //Solution
        DVec<c_float> solution_;

        //Var for P_PATH, q_PATH
        DVec<c_float> _cPATH;

        //int _nSegment;
        int _iterationsBetweenSEG;
        double _tk;
        int _dimVar;
        int _dimEq;
        int _nSpline;

        Vec2<float> initPoint, initVel, initAcc;
        Vec2<float> finalPoint, finalVel, finalAcc;
        Mat16<double> _T, _Tdot, _Tddot;

};
#endif