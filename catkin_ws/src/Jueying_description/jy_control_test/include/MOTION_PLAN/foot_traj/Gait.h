#ifndef PROJECT_GAIT_H
#define PROJECT_GAIT_H

#include <string>

#include "cppTypes.h"

using Eigen::Array4f;
using Eigen::Array4i;

namespace legID_P{
    constexpr size_t LF = 0;
    constexpr size_t LB = 1;
    constexpr size_t RB = 2;
    constexpr size_t RF = 3;
}//counter-clockwise

struct FootStateData{
    Vec2<float> footLocation[4];
    bool isContact[4];
};

class Gait{
    public:
        Gait(int nSegment, Vec41<int> offsets, Vec41<int> durations, const std::string& name);
        ~Gait();

        Vec41<float> getContactState();
        Vec41<float> getSwingState();
        const DMat<int> & getGaitPatternMat();
        void setIterations(int iterationsBetweenSEG, int currentIteration);
        float getCurrentStanceTime(float dtSEG, int leg);
        float getCurrentSwingTime(float dtSEG, int leg);
        float getCurrentGaitPhase();

    private:
        int* _gait_pattern;
        DMat<int> _gait_patternMat;
        Array4i _offsets;
        Array4i _durations;
        Array4f _offsetsFloat;//0 to 1
        Array4f _durationsFloat;//0 to 1
        int _stance;
        int _swing;
        int _iteration;
        int _nIterations;
        float _phase;

        std::string _name;
};

#endif