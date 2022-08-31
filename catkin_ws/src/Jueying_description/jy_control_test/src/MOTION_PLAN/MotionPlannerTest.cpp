#pragma GCC optimize(2)
#include "ros/ros.h"
#include "cppTypes.h"

#include "time.h"

#include "utility.h"
#include "UserParameter.h"
#include "ControlFSMData.h"
#include "MotionPlanner.h"
#include "MotionPlanData.h"

int main(int argc, char**argv)
{
    ros::init(argc, argv, "jy_control_test_node");
    ros::NodeHandle n;

    //Class MotionPlanner________________________________
    int nSegment(40);
    int iterationsBetweenSEG(3);
    double tf(0.03);
    UserParameter<float> paramf;
    UserParameter<double> paramd;
    ControlFSMData<float> data;
    DesMotionData DesResult; 

    //_______________________FIRST_RUN_____________________________________
    //Body State
    //Gait:20
    data.bodyStateEst.contactEstimate_P << 1, 1, 1, 1; //LF, LB, RB, RF
    data.bodyStateEst.position << 0., 0., 0.38;
    data.bodyStateEst.vBody << 0., 0., 0.;
    data.bodyStateEst.omegaBody << 0.,0.,0.;
    data.bodyStateEst.rpy << 0., 0., 0.;
    data.bodyStateEst.RotationMat = rpyTORotateMat(data.bodyStateEst.rpy[0],
                                                   data.bodyStateEst.rpy[1],
                                                   data.bodyStateEst.rpy[2]);
    //Leg State
    //LF
    data.legStateEst_P[0].q << 0.11, -0.93, 1.87;
    data.legStateEst_P[0].qd << 0, 0, 0;
    //LB
    data.legStateEst_P[1].q << 0.11, -0.93, 1.87;
    data.legStateEst_P[1].qd << 0, 0, 0;
    //RB 
    data.legStateEst_P[2].q << 0.11, -0.93, 1.87;
    data.legStateEst_P[2].qd << 0, 0, 0;
    //RF
    data.legStateEst_P[3].q << 0.11, -0.93, 1.87;
    data.legStateEst_P[3].qd << 0, 0, 0;


    MotionPlanner motionPlanner(nSegment, iterationsBetweenSEG, tf, paramf, paramd);
    // motionPlanner.iterationCounter_ = 3*6;
    
    //Body State
    //Gait:17
    // data.bodyStateEst.contactEstimate_P << 1, 1, 1, 0; //LF, LB, RB, RF
    // data.bodyStateEst.position << 0., 0., 0.;
    // data.bodyStateEst.vBody << 0., 0., 0.;
    // data.bodyStateEst.omegaBody << 0.,0.,0.;
    // data.bodyStateEst.rpy << 0., 0., 0.;
    // data.bodyStateEst.RotationMat = rpyTORotateMat(data.bodyStateEst.rpy[0],
    //                                                                                                          data.bodyStateEst.rpy[1],
    //                                                                                                          data.bodyStateEst.rpy[2]);
    // //Leg State
    // //LF
    // data.legStateEst_P[0].q << -0.14, -0.37, 0.85;
    // data.legStateEst_P[0].qd << 0., 0., 0.;
    // //LB
    // data.legStateEst_P[1].q << -0.14, -0.37, 0.85;
    // data.legStateEst_P[1].qd << 0., 0., 0.;
    // //RB
    // data.legStateEst_P[2].q << -0.14, -0.37, 0.85;
    // data.legStateEst_P[2].qd << 0., 0., 0.;
    // //RF
    // data.legStateEst_P[3].q << -0.14, -0.37, 0.85;
    // data.legStateEst_P[3].qd << 0., 0., 0.;


    // MotionPlanner motionPlanner(nSegment, iterationsBetweenSEG, tf, paramf, paramd);
    // motionPlanner.iterationCounter_ = 17;    

    motionPlanner.initialize();
    // ROS_INFO("____________MOTION_PLANNER_INITIALIZE DONE!___________");

    // motionPlanner._SetupCommand(data);
    // ROS_INFO("____________SETUP_COMMAND DONE!___________");
    clock_t startTime, endTime;
    startTime = clock();
    motionPlanner.run(data,DesResult);
    ROS_INFO("_________________________first________run______________________________");
    // motionPlanner.run(data,DesResult);
    // ROS_INFO("_________________________second________run______________________________");
    

    
    // //_______________________SECOND_RUN_____________________________________
    // //Body State
    // //Gait:19

    // data.bodyStateEst.contactEstimate_P << 1, 1, 1, 1; //LF, LB, RB, RF
    // data.bodyStateEst.position << 0., 0., 0.;
    // data.bodyStateEst.vBody << 0., 0.5, 0.;
    // data.bodyStateEst.omegaBody << 0.,0.,0.;
    // data.bodyStateEst.rpy << 0., 0., 0.;
    // data.bodyStateEst.RotationMat = rpyTORotateMat(data.bodyStateEst.rpy[0],
    //                                                                                                          data.bodyStateEst.rpy[1],
    //                                                                                                          data.bodyStateEst.rpy[2]);

    // motionPlanner.iterationCounter_ = 19;


    // startTime = clock();
    // motionPlanner.run(data);
    // endTime = clock();
    // ROS_INFO_STREAM("______________________SecondRun_TOTAL TIME: _____________________________\n"<<((double)(endTime - startTime)));
    //ROS_INFO_STREAM("clock_per_second" << CLOCKS_PER_SEC);
    //ROS_INFO("________________RUN_MOTION_PLAN DONE!____________________________");

    return 0;
}