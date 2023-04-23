#include <queue>
#include "ocs2_wbc/SimpleMotion/TerrainEstimator.h"
#include <iostream>

#define USE_PAST_DATA_NUM 2
#define NZEROS 2
#define NPOLES 2
#define GAIN   1.140829091e+04

namespace ocs2{
namespace wbc{
using namespace legged_robot;

scalar_t xv_a[NZEROS+1]={0}, yv_a[NPOLES+1]={0};
scalar_t xv_b[NZEROS+1]={0}, yv_b[NPOLES+1]={0};
scalar_t xv_d[NZEROS+1]={0}, yv_d[NPOLES+1]={0};

scalar_t filterloop(scalar_t input, scalar_t* xv, scalar_t* yv);
vector3_t filterloop(vector3_t input, std::vector<vector3_t>& xv, std::vector<vector3_t>& yv);
TerrainEstimator::TerrainEstimator(){
    estSlopeParam << 0., 0., 1.;
    xv_lf.resize(NZEROS+1);yv_lf.resize(NPOLES+1);
    xv_lh.resize(NZEROS+1);yv_lh.resize(NPOLES+1);
    xv_rf.resize(NZEROS+1);yv_rf.resize(NPOLES+1);
    xv_rh.resize(NZEROS+1);yv_rh.resize(NPOLES+1);

    for (auto &v:xv_lf)
        v = vector3_t::Zero();
    for (auto &v:xv_lh)
        v = vector3_t::Zero();
    for (auto &v:xv_rf)
        v = vector3_t::Zero();
    for (auto &v:xv_rh)
        v = vector3_t::Zero();
    for (auto &v:yv_lf)
        v = vector3_t::Zero();
    for (auto &v:yv_lh)
        v = vector3_t::Zero();
    for (auto &v:yv_rf)
        v = vector3_t::Zero();
    for (auto &v:yv_rh)
        v = vector3_t::Zero();

    
    // estSlopeParam_last << 0., 0., 1.;
}

TerrainEstimator::~TerrainEstimator(){}


vector3_t TerrainEstimator::run(const vector3_t &lf,
                                    const vector3_t &lh,
                                    const vector3_t &rf,
                                    const vector3_t &rh,
                                    const Eigen::Matrix<bool, 4, 1> &contact){

    // std::cout << "contact:" << contact.transpose() << std::endl;
    if(contact[0])
        lfPosQueue.insert(lfPosQueue.begin(),filterloop(lf,xv_lf,yv_lf));
    if(contact[1])
        lhPosQueue.insert(lhPosQueue.begin(),filterloop(lh,xv_lh,yv_lh));
    if(contact[2])
        rfPosQueue.insert(rfPosQueue.begin(),filterloop(rf,xv_rf,yv_rf));
    if(contact[3])
        rhPosQueue.insert(rhPosQueue.begin(),filterloop(rh,xv_rh,yv_rh));

    if(lfPosQueue.size() > USE_PAST_DATA_NUM)
        lfPosQueue.pop_back();
    if(lhPosQueue.size() > USE_PAST_DATA_NUM)
        lhPosQueue.pop_back();
    if(rfPosQueue.size() > USE_PAST_DATA_NUM)
        rfPosQueue.pop_back();
    if(rhPosQueue.size() > USE_PAST_DATA_NUM)
        rhPosQueue.pop_back();

    if(lfPosQueue.size() == USE_PAST_DATA_NUM && 
            lhPosQueue.size() == USE_PAST_DATA_NUM && 
                rfPosQueue.size() == USE_PAST_DATA_NUM && 
                    rhPosQueue.size() == USE_PAST_DATA_NUM ){

        // std::cout << "lfPosQueue: \n" << lfPosQueue[0].transpose() << std::endl 
        //     << lfPosQueue[1].transpose() << std::endl;
        // std::cout << "lhPosQueue: \n" << lhPosQueue[0].transpose() << std::endl 
        //     << lhPosQueue[1].transpose() << std::endl;
        // std::cout << "rfPosQueue: \n" << rfPosQueue[0].transpose() << std::endl 
        //     << rfPosQueue[1].transpose() << std::endl;
        // std::cout << "rhPosQueue: \n" << rhPosQueue[0].transpose() << std::endl 
        //     << rhPosQueue[1].transpose() << std::endl;

        Eigen::Matrix<scalar_t, USE_PAST_DATA_NUM*4, 3> H;
        Eigen::Matrix<scalar_t, USE_PAST_DATA_NUM*4, 1> I = matrix_t::Constant(USE_PAST_DATA_NUM*4,1,1);
        Eigen::Matrix<scalar_t, USE_PAST_DATA_NUM*4, 1> Z = matrix_t::Constant(USE_PAST_DATA_NUM*4,1,1);
        for (uint i = 0; i < USE_PAST_DATA_NUM; i++){
            H.row(i)                     = -lfPosQueue[i];
            H.row(i+USE_PAST_DATA_NUM)   = -lhPosQueue[i];
            H.row(i+2*USE_PAST_DATA_NUM) = -rfPosQueue[i];
            H.row(i+3*USE_PAST_DATA_NUM) = -rhPosQueue[i];
        }
        Z = -H.col(2); // "-" is very important 
        H.col(2) = I;

        // std::cout << "H:\n" << H << "\n";
        // std::cout << "Z:\n" << Z << "\n";
        

        Eigen::JacobiSVD<matrix_t> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // not sure if we need to svd.sort()... probably not
        int const nrows(svd.singularValues().rows());
        matrix_t invS;
        invS = matrix_t::Zero(nrows, nrows);
        const scalar_t sigmaThreshold = 0.0000001;
        for (int ii(0); ii < nrows; ++ii) {
            if (svd.singularValues().coeff(ii) > sigmaThreshold) {
            invS.coeffRef(ii, ii) = 1.0 / svd.singularValues().coeff(ii);
            } else {
            // invS.coeffRef(ii, ii) = 1.0/ sigmaThreshold;
            printf("terrain sigular value is too small: %f\n",svd.singularValues().coeff(ii));
            }
        }
        matrix_t H_invMatrix = svd.matrixV() * invS * svd.matrixU().transpose();
        
        estParam =  H_invMatrix * Z; // estParams = [a b -d]

        // vector3_t currentResult = estParam;

        // estSlopeParam = currentResult;
        // std::cout <<"estTerr!!!\n";
        // estSlopeParam = lowpass_cof * currest + (1 - lowpass_cof) * estSlopeParam_last;
        // estSlopeParam_last = estSlopeParam;
        estSlopeParam[0] = filterloop(estParam[0], xv_a, yv_a);
        estSlopeParam[1] = filterloop(estParam[1], xv_b, yv_b);
        estSlopeParam[2] = filterloop(-estParam[2], xv_d, yv_d); // -d -> d

        // estSlopeParam[0] = estParam[0] ;
        // estSlopeParam[1] = estParam[1] ;
        // estSlopeParam[2] = -estParam[2]; // -d -> d
        // estSlopeParam[1] = filterloop(currest[1]);   
        return estSlopeParam;
    }
    else{
        std::cout << ">>>>>>>>>>>>>>>>>>not enough data to estimate terrain slope<<<<<<<<<<<<<<<<<<<" << std::endl;
        return estSlopeParam;
    }
}



/* Digital filter designed by mkfilter/mkshape/gencode   A.J. Fisher
   Command line: ./mkfilter -Ch -0.5 -Lp -o 4 -a 0.001 -l */


scalar_t filterloop(scalar_t input, scalar_t* xv, scalar_t* yv)
  { 
// xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; xv[4] = xv[5]; 
//         xv[5] = input / GAIN;
//         yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; yv[4] = yv[5]; 
//         yv[5] =   (xv[0] + xv[5]) + 5 * (xv[1] + xv[4]) + 10 * (xv[2] + xv[3])
//                      + (  0.8585472563 * yv[0]) + ( -4.4236578665 * yv[1])
//                      + (  9.1191966574 * yv[2]) + ( -9.4015942664 * yv[3])
//                      + (  4.8475080037 * yv[4]);
        // std::cout << "GAIN:" << GAIN << "\nin:" << xv[4] << "\n";
        xv[0] = xv[1]; xv[1] = xv[2]; 
        xv[2] = input / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; 
        yv[2] =   (xv[0] + xv[2]) + 2 * xv[1]
                     + ( -0.9736948720 * yv[0]) + (  1.9733442498 * yv[1]);

        return yv[2];
      
  }

vector3_t filterloop(vector3_t input, std::vector<vector3_t>& xv, std::vector<vector3_t>& yv)
  { 
        xv[0] = xv[1]; xv[1] = xv[2]; 
        xv[2] = input / GAIN;
        yv[0] = yv[1]; yv[1] = yv[2]; 
        yv[2] =   (xv[0] + xv[2]) + 2 * xv[1]
                     + ( -0.9736948720 * yv[0]) + (  1.9733442498 * yv[1]);
        return yv[2];
  }

// template <typename T>
// T Filter<T>::run(T input){
//         xv[0] = xv[1]; xv[1] = xv[2]; xv[2] = xv[3]; xv[3] = xv[4]; 
//         xv[4] = input / gain;
//         yv[0] = yv[1]; yv[1] = yv[2]; yv[2] = yv[3]; yv[3] = yv[4]; 
//         yv[4] =   (xv[0] + xv[4]) + 4 * (xv[1] + xv[3]) + 6 * xv[2]
//                      + ( -0.9925048376 * yv[0]) + (  3.9774471147 * yv[1])
//                      + ( -5.9773794638 * yv[2]) + (  3.9924371861 * yv[3]);
//         std::cout << "GAIN:" << gain << "\nin:" << xv[4] << "\n";

//         return yv[4];
// }
}
}
