#ifndef SEARCHOPTCOEFFICIENTS_H
#define SEARCHOPTCOEFFICIENTS_H

#include "cppTypes.h"
#include "SupportPolygon.h"
#include "UserParameter.h"

#include "osqp.h"

#include <vector>

class SearchOptCoefficients{
    public:
        SearchOptCoefficients(int nSegment, int iterationsBetweenSEG, double tfk, SupportPolygon* data, UserParameter<double> & param);
        ~SearchOptCoefficients();

        void run(c_float * P_x, c_float * q,
                 const Vec2<float> & initPoint, const Vec2<float> &initVel, const Vec2<float>& initAcc,
                 double zcom, const Vec2<float> & finalPoint,
                 DVec<double> & solution, bool &if_solved);//TODO：支撑多边形没有重叠时的代码测试
                           
        Mat2_12<double>  _timePosMat(double time);
        Mat2_12<double>  _timeVelMat(double time);
        Mat2_12<double>  _timeAccMat(double time);
        Mat16<double>  _Eta(double time);
        Mat16<double>  _Eta_dd(double time);
    
    protected:
        int _getNumOfLine(int numVertice);
        void LeftCrossRightCross(int SplineNum);
        void LeftCrossRightNoCross(int SplineNum);
        void LeftNoCrossRightCross(int SplineNum);
        void LeftNoCrossRightNoCross(int SplineNum);

        void LastSplineCross();
        void LastSplineNoCross();

        //Var of EqConstraints
        DVec<c_float> A_x_0;
        DVec<c_int> A_i_0;
        DVec<c_int> A_p_0;
        c_int A_nnz_0;
        c_int A_nnz_0_half;

        DVec<c_float> A_x_k;
        DVec<c_int> A_i_k;
        Vec12<c_int> A_p_k;
        c_int A_nnz_k;

        int num_line;
        vectorAligned<Vec31<double>> line;

        int Index_Spline;
        int Index_Eq;
        int Index_InEq;
        int Index_A_x_;
        int Index_A_i_;
        int Index_A_p_;

        //Var for QP
        //P
        c_int P_nnz_;
        c_int* P_i_;
        c_int* P_p_;

        //A
        c_int A_nnz_;
        DVec<c_float> l_vec;
        DVec<c_float> u_vec;
        //n,m
        c_int n;
        c_int m;

        // OSQPWorkspace *work;
        // OSQPSettings *settings;
        // OSQPData *data;

        // int _nSegment;
        int _iterationsBetweenSEG;
        int _nSpline;
        int _dimVar;
        int _dimEq;
        int _dimInEq;
        SupportPolygon*  _poly_data;
        double _tfk;
        double _Xmax, _Xmin, _Ymax, _Ymin;
        float _margin;

        //Input Var
        double zcom_;
};

#endif