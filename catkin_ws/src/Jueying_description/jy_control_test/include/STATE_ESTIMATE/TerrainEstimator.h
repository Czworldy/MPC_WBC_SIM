#ifndef TERRAINESTIMATOR_H
#define TERRAINESTIMATOR_H

#include <vector>
#include "cppTypes.h"

class TerrainEstimator
{
private:
    std::vector<Vec31<float>> lfPosQueue;
    std::vector<Vec31<float>> rfPosQueue;
    std::vector<Vec31<float>> lhPosQueue;
    std::vector<Vec31<float>> rhPosQueue;

    Vec31<float> estSlopeParam;
    // Vec31<float> estSlopeNormal_last;
    Vec31<float> estParam;

    std::vector<Vec31<float>> xv_lf, yv_lf;
    std::vector<Vec31<float>> xv_lh, yv_lh;
    std::vector<Vec31<float>> xv_rf, yv_rf;
    std::vector<Vec31<float>> xv_rh, yv_rh;

    // const float lowpass_cof = 0.02;

public:
    TerrainEstimator();
    ~TerrainEstimator();

    Vec31<float> run(const Vec31<float> &lf,
                                        const Vec31<float> &lh,
                                        const Vec31<float> &rf,
                                        const Vec31<float> &rh,
                                        const Vec41<int> &contact);
    
    Vec41<float> getFeetHeight(){ 
        Vec41<float> feetHeight;
        if(lfPosQueue.empty() || rfPosQueue.empty() || lhPosQueue.empty() || rhPosQueue.empty())
            return Vec41<float>::Zero();
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


#endif

