// #pragma GCC optimize(2)
#include "quadruped_dynamics_model.h"
#include "ros/ros.h"
#include "task.h"

#include "wbc_ctrl.h"
#include "time.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "jy_control_test_node");
    ros::NodeHandle n;

    
 
    UserParameter<float> paramf;
    LocomotionCtrlData<float> _wbc_data;
    ControlFSMData<float> _data;
    WBC_Ctrl<float> *_wbc_ctrl;

    

    for(int i(0); i<100; i++){
        

    _wbc_data.pBody_des <<  0., 0.8, 0.5;
    _wbc_data.vBody_des << 0., 0.1, 0.;
    _wbc_data.aBody_des << 0.,1.,0.;
    _wbc_data.pBody_RPY_des << 0.2,0.1,0.;
    
    _wbc_data.pFoot_des << 0, 0.2,0.6;
    _wbc_data.vFoot_des << 0.,0.,0.;
    _wbc_data.aFoot_des << 0.,0.,0.;
    _wbc_data.contact_state << 0, 1, 0, 1;

    _data.bodyStateEst.contactEstimate << 0,1,0,1;
    _data.bodyStateEst.position << 0., 0.7, 0.4;
    _data.bodyStateEst.vBody << 0,0.05,0;
    _data.bodyStateEst.orientation.w()=1;
    _data.bodyStateEst.orientation.x()=0;
    _data.bodyStateEst.orientation.y()=0;
    _data.bodyStateEst.orientation.z()=0;
    _data.bodyStateEst.omegaBody << 0,0,0.1;

    _data.legStateEst[0].q << 0., -0.44, 1.44;
    _data.legStateEst[0].qd << 0., 0, 0;
    _data.legStateEst[1].q << 0., -0.44, 1.44;
    _data.legStateEst[1].qd << 0., 0, 0;
    _data.legStateEst[2].q << 0., -0.44, 1.44;
    _data.legStateEst[2].qd << 0., 0, 0;
    _data.legStateEst[3].q << 0., -0.44, 1.44;
    _data.legStateEst[3].qd << 0., 0, 0;

    //ROS_INFO("___________________Set Param Done!_____________________________________");

    clock_t startTime, endTime;
    startTime = clock();
    QuadrupedDynamicsModel jueying;
_wbc_ctrl = new WBC_Ctrl<float>(&jueying);
    DVec<float> tau;
    _wbc_ctrl->run(&_wbc_data, _data,tau);
    endTime = clock();
    ROS_INFO_STREAM("TIME_RUN______"<<((double)(startTime - endTime)));
    //ROS_INFO("___________________WBC_Ctrl RESULT_____________________________________");

        delete _wbc_ctrl;
    }



    return 0;
}