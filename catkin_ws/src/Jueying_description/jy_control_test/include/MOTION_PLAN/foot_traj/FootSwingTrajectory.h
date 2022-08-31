#ifndef FOOTSWINGTRAJECTORY_H
#define FOOTSWINGTRAJECTORY_H

#include "cppTypes.h"

template<typename T>
class FootSwingTrajectory{
    public:
        FootSwingTrajectory(){
            _p0.setZero();
            _pf.setZero();
            _p.setZero();
            _v.setZero();
            _a.setZero();
            _height = 0;
        }
        ~FootSwingTrajectory(){}

        //Set the starting location of the foot
        void setInitialPosition(Vec31<T> p0){
            _p0 = p0;
        }

        //Set the desired final position of the foot
        void setFinalPosition(Vec31<T> pf){
            _pf = pf;
        }

        //Set the maximum height of the swing
        void setHeight(T h){
            _height = h;
        }

        void computeSwingTrajectoryBezier(T phase, T swingTime);

        Vec31<T> getPosition(){return _p;}
        Vec31<T> getVelocity(){return _v;}
        Vec31<T> getAcceleration(){return _a;}

    private:
        Vec31<T> _p0, _pf, _p, _v, _a;
        T _height;
};
#endif