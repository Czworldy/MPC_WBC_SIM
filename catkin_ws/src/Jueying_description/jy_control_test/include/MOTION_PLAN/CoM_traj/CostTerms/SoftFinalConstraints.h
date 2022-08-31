#ifndef SOFTFINALCONSTRAINTS_H
#define SOFTFINALCONSTRAINTS_H

#include "cppTypes.h"
#include "UserParameter.h"
#include "osqp.h"

class SoftFinalConstraints{
    public:
        SoftFinalConstraints(int nSegment, int iterationsBetweenSEG, double tk, UserParameter<double>& param);
        ~SoftFinalConstraints(){}

        void UpdateCostFunction(const Vec2<float> & pf, 
                                c_float *P_Final_x_, c_float *q_Final_);
    
    protected:
        
        DVec<c_float> _cFinal;
        //int _nSegment;
        int _iterationsBetweenSEG;
        double _tf;
        int _nSpline;
        Vec2<double> _wf;
        Mat16<c_float> Tf;
};

#endif