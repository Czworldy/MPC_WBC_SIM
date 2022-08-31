#ifndef ACCELERATIONMIN_H
#define ACCELERATIONMIN_H

#include "osqp.h"
#include "cppTypes.h"

class AccelerationMin{
    public:
        AccelerationMin(int nSegment, int iterationsBetweenSEG, double tk);
        ~AccelerationMin(){}

        void UpdateCostFunction(c_float *Q_x, c_int*Q_i, c_int *Q_p);//for PathRegularization
        void UpdateCostFunctionAcc(c_float *Q_x);//for SearchOptCoefficients
    
    protected:
        //int _nSegment;
        int _iterationsBetweenSEG;
        double _tk;
        int _nSpline;
};

#endif