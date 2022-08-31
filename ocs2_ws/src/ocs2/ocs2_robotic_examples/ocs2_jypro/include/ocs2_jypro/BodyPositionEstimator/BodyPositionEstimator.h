#pragma once

#include "cppTypes.h"
#include "SystemParameter.h"
#include "ocs2_core/Types.h"
namespace ocs2 {
class BodyPositionEst{
    public:
        BodyPositionEst(){_body_height_pre = 0.45;};
        ~BodyPositionEst(){};

        Vec31<double> bodyPositionEst(const Vec31<double> & qleg_lf,
                                     const Vec31<double> & qleg_lb,
                                     const Vec31<double> & qleg_rf,
                                     const Vec31<double> & qleg_rb,
                                     const Vec41<double>& contactState);

        Vec31<double> bodyPositionEst_P(const Vec31<double> & qleg_lf_P,
                                       const Vec31<double> & qleg_lb_P,
                                       const Vec31<double> & qleg_rb_P,
                                       const Vec31<double> & qleg_rf_P,
                                       const Vec41<double>& contactState_P);

        Vec2<double> getPolygonCenter(const vectorAligned<Vec2<double>>&  polygon);
    
        Vec31<double> _foot_lf_in_center, _foot_lb_in_center, _foot_rf_in_center, _foot_rb_in_center;

    private:
        Vec31<double> _pos_body;//body position in foot print center frame
        Vec31<double> _foot_lf, _foot_lb, _foot_rf, _foot_rb;

        Vec2<double> _foot_print_center;
        SystemParameter<double> _sysParam;
        double _body_height;

        double _body_height_pre;

};

class QuaternionToRPY
{

public:
    int circle_counter = 0;
    scalar_t preYaw = 0;

    Vec31<scalar_t> quaternionToTotalRad(const Eigen::Quaternion<scalar_t>& q){
        Vec31<scalar_t> rpy;
        rpy[0] = atan2(2*(q.w()*q.x()+q.y()*q.z()), 1-2*(pow(q.x(), 2)+pow(q.y(), 2)));
        rpy[1] = asin(2*(q.w()*q.y() - q.z()*q.x()));
        rpy[2] = atan2(2*(q.w()*q.z() + q.x()*q.y()), 1 - 2*(pow(q.y(), 2)+pow(q.z(), 2)));

        scalar_t deltaYaw = rpy[2] - preYaw;
        if(deltaYaw > 1.6*M_PI){
            circle_counter--;
        }
        else if(deltaYaw < -1.6*M_PI){
            circle_counter++;
        }
        preYaw = rpy[2];
        rpy[2] = rpy[2] + 2*circle_counter*M_PI;

        return rpy;
    }
    
    void reset(){circle_counter = 0; preYaw = 0;}

    QuaternionToRPY(){reset();}
};
} //namespace ocs2

