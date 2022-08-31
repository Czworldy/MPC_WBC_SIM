#ifndef DEVIATIONFROMPRE_H
#define DEVIATIONFROMPRE_H

#include "cppTypes.h"
#include "UserParameter.h"
#include "osqp.h"

class DeviationFromPre{
    public:
        DeviationFromPre(int nSegment, int iterationsBetweenSEG, double tfk, UserParameter<double> param);
        ~DeviationFromPre();

        void UpdateCostFunction(const DVec<c_float> & preSolution,
                                c_float* P_DEVIA_x_, c_float* q_DEVIA_);
    
    protected:
        DVec<double> _W;
        DVec<double> _Wdot;
        DVec<double> _Wddot;

        Mat16<c_float> _T;
        Mat16<c_float> _Tdot;
        Mat16<c_float> _Tddot;

        Mat16<c_float> _Tpre;
        Mat16<c_float> _Tpre_dot;
        Mat16<c_float> _Tpre_ddot;

        DVec<c_float> _cDEVIA;

        // int _nSegment;
        int _iterationsBetweenSEG;
        double _tfk;
        int _nSpline;
        int _dimVar;
        double _t_control_loop;
};

#endif