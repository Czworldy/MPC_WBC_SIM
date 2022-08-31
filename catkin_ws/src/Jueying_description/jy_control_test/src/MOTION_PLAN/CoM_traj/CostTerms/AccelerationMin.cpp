#include "AccelerationMin.h"

AccelerationMin::AccelerationMin(int nSegment, int iterationsBetweenSEG, double tk):
    //_nSegment(nSegment),
    _iterationsBetweenSEG(iterationsBetweenSEG),
    _tk(tk),
    _nSpline(nSegment)
    {

}

void AccelerationMin::UpdateCostFunction(c_float *Q_x, c_int*Q_i, c_int *Q_p){

    // c_float Q_x[24*_nSpline]; //Vector of data
    // c_int Q_nnz = 24*_nSpline;//Maximum number of entries
    // c_int Q_i[24*_nSpline]; //Row indices
    // c_int Q_p[12*_nSpline+1];//Column pointers

    c_float Q_x_k[12] = {(400./7.)*pow(_tk, 7) + 10E-7,
                        40. *pow(_tk, 6) + 10E-7,
                        28.8*pow(_tk, 5) + 10E-7,
                        24. *pow(_tk, 5) + 10E-7,
                        18. *pow(_tk, 4) + 10E-7,
                        12. *pow(_tk, 3) + 10E-7,
                        10. *pow(_tk, 4) + 10E-7,
                        8. *pow(_tk, 3) + 10E-7,
                        6. *pow(_tk, 2) + 10E-7,
                        4. *_tk + 10E-7,
                        10E-7,
                        10E-7};
    c_int Q_i_k[12] = {0, 0, 1, 0, 1, 2, 0, 1, 2, 3, 4, 5};
    c_int Q_p_k[6] = {0, 1, 3, 6, 10, 11};

    for(int i(0); i<(2*_nSpline); i++){
        //Q_x
        for(int j(0); j<12; j++)
            Q_x[j+12*i] = Q_x_k[j];
        //Q_i_k
        for(int j(0); j<12; j++)
            Q_i[j+12*i] = Q_i_k[j] + 6*i;
        //Q_p_k
        for(int j(0); j<6;j++)
            Q_p[j+6*i] = Q_p_k[j] + 12*i;
    }
    Q_p[12*_nSpline] = 24*_nSpline;
}

void AccelerationMin::UpdateCostFunctionAcc(c_float *Q_x){
    
    c_float Q_x_k[21] = {(400./7.)*pow(_tk, 7)+10E-7,
                                    40. *pow(_tk, 6)+10E-7,
                                    28.8*pow(_tk, 5)+10E-7,
                                    24. *pow(_tk, 5)+10E-7,
                                    18. *pow(_tk, 4)+10E-7,
                                    12. *pow(_tk, 3)+10E-7,
                                    10. *pow(_tk, 4)+10E-7,
                                    8. *pow(_tk, 3)+10E-7,
                                    6. *pow(_tk, 2)+10E-7,
                                    4. *_tk+10E-7,
                                    10E-7,10E-7,10E-7,10E-7,
                                    10E-7,
                                    10E-7,10E-7,10E-7,10E-7,10E-7,
                                    10E-7};
    for(int i(0); i<(2*_nSpline);i++){
        for(int j(0); j<21; j++)
            Q_x[j+21*i] = Q_x_k[j];
    }
}