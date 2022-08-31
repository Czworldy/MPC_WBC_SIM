#include "quadruped_dynamics_model.h"
#include "ros/ros.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "jy_control_test_node");
    ros::NodeHandle n;
    QuadrupedDynamicsModel jueying;

    FBModelState<double> state_;
    state_.bodyOrientation.w() = 1;
    state_.bodyOrientation.x() = 0;
    state_.bodyOrientation.y() = 0;
    state_.bodyOrientation.z() = 0  ;
    state_.bodyPosition << 0., 0., 0.;
    state_.bodyVelocity << 0., 0., 0., 0., 0., 0.;
    state_.q_leg <<  0.0, -0.95, 1.7,0.0, -0.95, 1.7,0.0, -0.95, 1.7,0.0, -0.95, 1.7;
    // state_.q_leg <<  0.0, 0.0, 0.0,
    //                                  0.0, 0.0, 0.0,
    //                                  0.0, 0.0, 0.0,
    //                                  0.0, 0.0, 0.0;
    state_.qd_leg <<  0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.;
    state_.contact_state_ <<1, 1, 1, 1;

    jueying.setState(state_);

    DVec<double> foot_position_lf,foot_position_lb,foot_position_rf,foot_position_rb, foot_vel, _N; 
    DMat<double> _H, _CJ, _JCOM, _JSF;
    jueying.massMatrix();
    jueying.nonlinearEffect();
    jueying.contactJacobian();
    jueying.CoM6DJacobian();
   
    foot_position_lf = jueying.swingFootPosition(legID::LF);
    foot_position_lb = jueying.swingFootPosition(legID::LB);
    foot_position_rf = jueying.swingFootPosition(legID::RF);
    foot_position_rb = jueying.swingFootPosition(legID::RB);
    foot_vel = jueying.swingFootVelocity(legID::LF);
    _H =  jueying.getMassMatrix();
    _N = jueying.getNolinearEffect();
    _CJ = jueying.getContactJacobian();
    _JCOM = jueying.getCoM6DJacobian();
    _JSF =  jueying.swingFootJacobian(legID::LF);
    ROS_INFO_STREAM("LF_FOOT_VELOCITY:   " << foot_vel);
    ROS_INFO_STREAM("MASSMATRIX:   " <<  _H);
    ROS_INFO_STREAM("NONLINEAREFFECT:   " << _N);
    ROS_INFO_STREAM("CONTACTJACOBIAN:   " <<_CJ);
    ROS_INFO_STREAM("COMJACOBIAN:   " <<_JCOM);
    ROS_INFO_STREAM("SWINGFOOTJACOBIAN:   " <<_JSF);
    ROS_INFO_STREAM("LF_FOOT_POSITION:   " <<foot_position_lf);
    ROS_INFO_STREAM("LB_FOOT_POSITION:   " <<foot_position_lb);
    ROS_INFO_STREAM("RF_FOOT_POSITION:   " <<foot_position_rf);
    ROS_INFO_STREAM("RB_FOOT_POSITION:   " <<foot_position_rb);

    

    return 0;
}