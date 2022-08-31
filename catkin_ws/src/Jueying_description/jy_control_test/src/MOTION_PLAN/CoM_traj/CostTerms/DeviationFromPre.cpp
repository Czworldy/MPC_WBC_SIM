#include "DeviationFromPre.h"
#include "time.h"
#include "ros/ros.h"

DeviationFromPre::DeviationFromPre(int nSegment, int iterationsBetweenSEG, 
                                   double tfk, UserParameter<double> param):
    //_nSegment(nSegment),
    _iterationsBetweenSEG(iterationsBetweenSEG),
    _tfk(tfk),
    _nSpline(nSegment)
    {
        _W = param.weighDeviation;
        _Wdot = param.weighDeviation_dot;
        _Wddot = param.weighDeviation_ddot;
        _t_control_loop = param.t_plan_control_loop;


        _dimVar = 12*_nSpline;
        _cDEVIA = DVec<c_float>::Zero(_dimVar);
}

DeviationFromPre::~DeviationFromPre(){
}

void DeviationFromPre::UpdateCostFunction(const DVec<c_float> & preSolution,
                                          c_float* P_DEVIA_x_, c_float* q_DEVIA_){
    double t;
    for(int i(0); i<_nSpline; i++ ){
        t = (i+1)*_tfk;
        //position
        _T <<  pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1.;
        //velocity
        _Tdot << 5*pow(t, 4), 4*pow(t, 3), 3*pow(t, 2), 2*t, 1, 0.;
        //acceleration
        _Tddot << 20*pow(t, 3), 12*pow(t, 2), 6*t, 2., 0., 0.;
    
        if(i<(_nSpline - 1))
            t = (i + 1) * _tfk + _t_control_loop;
        else
            t = _t_control_loop;
        //position
        _Tpre <<  pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1.;
        //velocity
        _Tpre_dot << 5*pow(t, 4), 4*pow(t, 3), 3*pow(t, 2), 2*t, 1, 0.;
        //acceleration
        _Tpre_ddot << 20*pow(t, 3), 12*pow(t, 2), 6*t, 2., 0., 0.;

        DMat<double> TT( _T.transpose()*_T);
        DMat<double> TT_dot(_Tdot.transpose()*_Tdot);
        DMat<double> TT_ddot(_Tddot.transpose()*_Tddot);

        DMat<double> TTpre( _T.transpose()*_Tpre);
        DMat<double> TTpre_dot(_Tdot.transpose()*_Tpre_dot);
        DMat<double> TTpre_ddot(_Tddot.transpose()*_Tpre_ddot);

        int index = 0;
        for(int j(0); j<6; j++){
            for(int k(0); k<(j+1); k++){
                P_DEVIA_x_[42*i+index] = _W[0]*TT(k,j) +_Wdot[0]* TT_dot(k,j) +_Wddot[0]* TT_ddot(k,j);
                P_DEVIA_x_[42*i+index+21] = _W[1]*TT(k,j) +_Wdot[1]* TT_dot(k,j) +_Wddot[1]* TT_ddot(k,j);
                index++;
            }
        }

        _cDEVIA.segment(12*i,6) =  - _W[0] * TTpre * preSolution.segment(12*i,6) 
                                                               - _Wdot[0] * TTpre_dot * preSolution.segment(12*i,6);
                                                               - _Wddot[0] * TTpre_ddot * preSolution.segment(12*i,6);
        _cDEVIA.segment(12*i+6,6) =  -_W[1] * TTpre * preSolution.segment(12*i+6,6);
                                                                    - _Wdot[1] * TTpre_dot * preSolution.segment(12*i+6,6);
                                                                    - _Wddot[1] * TTpre_ddot * preSolution.segment(12*i+6,6);
    }
    for(int i(0); i<_dimVar; i++)
        q_DEVIA_[i] = _cDEVIA[i];
}