#pragma once

#include <vector>
#include "ocs2_jypro/common/Types.h"
#include "ocs2_jypro/synchronized_module/TerrainReceiver.h"
namespace ocs2{
namespace wbc{
using namespace legged_robot;
class TerrainEstimator
{
private:
    std::vector<vector3_t> lfPosQueue;
    std::vector<vector3_t> rfPosQueue;
    std::vector<vector3_t> lhPosQueue;
    std::vector<vector3_t> rhPosQueue;

    vector3_t estSlopeParam;
    // vector3_t estSlopeNormal_last;
    vector3_t estParam;

    std::vector<vector3_t> xv_lf, yv_lf;
    std::vector<vector3_t> xv_lh, yv_lh;
    std::vector<vector3_t> xv_rf, yv_rf;
    std::vector<vector3_t> xv_rh, yv_rh;

    // const float lowpass_cof = 0.02;

public:
    TerrainEstimator();
    ~TerrainEstimator();

    vector3_t run(const vector3_t &lf,
                                        const vector3_t &lh,
                                        const vector3_t &rf,
                                        const vector3_t &rh,
                                        const Eigen::Matrix<bool, 4, 1> &contact);
    
    Eigen::Vector4d getFeetHeight(){ 
        Eigen::Vector4d feetHeight;
        if(lfPosQueue.empty() || rfPosQueue.empty() || lhPosQueue.empty() || rhPosQueue.empty())
            return Eigen::Vector4d::Zero();
        feetHeight << lfPosQueue.back().z(), lhPosQueue.back().z(), 
                        rfPosQueue.back().z(), rhPosQueue.back().z();
        return feetHeight;
    }


};

// template <typename T>
// class Filter
// {
// private:
//     uint8_t Nzero = 4;
//     uint8_t Npoles = 4;
//     const T gain = 2.718536922e+10;
//     T xv[Nzero+1], yv[Npoles+1];
// public:
//     Filter(){};
//     ~Filter(){};

//     T run(T input);
// };

// extern template class Filter<double>;
// extern template class Filter<float>;

}
}



