#include "Gait.h"

Gait::Gait(int nSegment, Vec41<int> offsets, Vec41<int> durations, const std::string& name):
    _offsets(offsets.array()),
    _durations(durations.array()),
    _nIterations(nSegment)
{
    _name = name;
    //allocate memory for gait pattern
    _gait_pattern = new int[nSegment*4];

    _offsetsFloat = offsets.cast<float>()/(float)nSegment;
    _durationsFloat = durations.cast<float>()/(float)nSegment;

    _stance  = durations[0];
    _swing = nSegment - durations[0];
}

Gait::~Gait()
{
    delete[] _gait_pattern;
}

Vec41<float> Gait::getContactState(){
    Array4f progress = _phase - _offsetsFloat;

    for(int i=0; i<4; i++){
        if(progress[i]<0) progress[i]+=1.;
        if(progress[i] > _durationsFloat[i])
        {
            progress[i] = 0.;
        }
        else
        {
            progress[i] = progress[i]/_durationsFloat[i];
        } 
    }
    return progress.matrix();
}

Vec41<float> Gait::getSwingState(){
    Array4f swing_offset = _offsetsFloat + _durationsFloat;
    for(int i=0; i<4; i++)
        if(swing_offset[i]>1) swing_offset-=1.;
    Array4f swing_duration = 1. - _durationsFloat;

    Array4f progress = _phase - swing_offset;

    for(int i=0; i<4; i++)
    {
        if(progress[i]<0) progress[i] += 1.f;
        if(progress[i]>swing_duration[i])
        {
            progress[i] = 0;
        }
        else
        {
            progress[i] = progress[i] / swing_duration[i];
        }
    }
    return progress.matrix();
}

const DMat<int> & Gait::getGaitPatternMat()
{
    _gait_patternMat.resize(4, _nIterations);
    for(int i = 0; i < _nIterations; i++)
    {
        int iter = (i + _iteration) % _nIterations;
        Array4i progress = iter - _offsets;
        for(int j=0; j<4; j++)
        {
            if(progress[j] < 0) progress[j] += _nIterations;
            if(progress[j] < _durations[j])
                _gait_patternMat(j,i)= 1;
            else
                _gait_patternMat(j,i) = 0;
            
        }
    }

    return _gait_patternMat;
}


void Gait::setIterations(int iterationsBetweenSEG, int currentIteration)
{
    _iteration = (currentIteration / iterationsBetweenSEG) % _nIterations;
    _phase = (float)(currentIteration % (iterationsBetweenSEG * _nIterations)) / (float) (iterationsBetweenSEG * _nIterations);
}

float Gait::getCurrentGaitPhase(){
    return _phase;
}

float Gait::getCurrentStanceTime(float dtSEG, int leg){
    (void) leg;
    return dtSEG * _stance;
}

float Gait::getCurrentSwingTime(float dtSEG, int leg){
    (void) leg;
    return dtSEG * _swing;
}
