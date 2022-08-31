#include "SoftFinalConstraints.h"
#include "ros/ros.h"

SoftFinalConstraints::SoftFinalConstraints(int nSegment, 
                                           int iterationsBetweenSEG, 
                                           double tk, 
                                           UserParameter<double>& param):
    //_nSegment(nSegment),
    _iterationsBetweenSEG(iterationsBetweenSEG),
    _nSpline(nSegment),
    _tf(tk*nSegment){

        _wf = param.weighFinalConst;
        _cFinal = DVec<c_float>::Zero(12);

}

 void SoftFinalConstraints::UpdateCostFunction(const Vec2<float> & pf, 
                                               c_float *P_Final_x_, c_float *q_Final_){
    Tf << pow(_tf,5), pow(_tf,4), pow(_tf,3), pow(_tf,2), _tf, 1.;
    
    Mat6<c_float> TT( Tf.transpose()*Tf);
    // ROS_INFO_STREAM("_tf: " << _tf);
    // ROS_INFO_STREAM("VECTOR_tf: \n" << Tf);
    // ROS_INFO_STREAM("FINAL_MATRIX:\n" << TT);
    int index = 0;
    for(int j(0); j<6; j++){
        for(int k(0); k<(j+1); k++){
            P_Final_x_[index] = _wf[0]*TT(k,j);
            P_Final_x_[index+21] = _wf[1]*TT(k,j);
            index++;
        }       
    }

    _cFinal.segment(0,6) =  - _wf[0] * pf[0] * Tf.transpose();
    _cFinal.segment(6,6) =  - _wf[1] * pf[1] * Tf.transpose();

    for(int i(0); i<12; i++)
        q_Final_[i] = _cFinal[i];
}