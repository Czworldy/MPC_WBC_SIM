//
// Created by czworldy on 2023/4/20.
//

#include "ocs2_wbc_ros/SingleWbcRos.h"
// #include <Boost>


#include "ocs2_wbc/TrackingQP.h"
#include "ocs2_wbc/HoQp.h"

namespace ocs2{
namespace wbc{
SingleWbcRos::SingleWbcRos(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                 const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                                 const std::string& paramFile, ros::NodeHandle& nh)
        : WbcBase(pinocchioInterface, info, eeKinematics, paramFile){
    taskWeight_ = vector_t::Ones(4);
    ros::NodeHandle nh_weight = ros::NodeHandle(nh, "wbc");

    Task constraints = formulateFloatingBaseEomTask() + formulateNoContactMotionTask()
                 + formulateTorqueLimitsTask() + formulateFrictionConeTask();

    qpPtr_ = std::make_shared<TrackingQP>(info.generalizedCoordinatesNum + 3 * info.numThreeDofContacts, constraints.a_.rows() + constraints.d_.rows());

    dynamic_srv_ = std::make_shared<dynamic_reconfigure::Server<ocs2_wbc_ros::wbcWeightConfig>>(nh_weight);
    // dynamic_reconfigure::Server<ocs2_wbc_ros::wbcWeightConfig>::CallbackType cb = [this](auto&& PH1, auto&& PH2) {
    //     dynamicCallback(std::forward<decltype(PH1)>(PH1), std::forward<decltype(PH2)>(PH2));
    // };
    dynamic_reconfigure::Server<ocs2_wbc_ros::wbcWeightConfig>::CallbackType cb;
    cb = boost::bind(&SingleWbcRos::dynamicCallback, this, _1, _2);
    dynamic_srv_->setCallback(cb);
}

void SingleWbcRos::dynamicCallback(ocs2_wbc_ros::wbcWeightConfig &config, uint32_t) {
    UserParameter& userParameters = getUserParam();
    userParameters.mu = config.frictionCoeff;

    userParameters.Kp_body.x() = config.Kp_body_x;
    userParameters.Kp_body.y() = config.Kp_body_y;
    userParameters.Kp_body.z() = config.Kp_body_z;

    userParameters.Kp_ori.x() = config.Kp_ori_x;
    userParameters.Kp_ori.y() = config.Kp_ori_y;
    userParameters.Kp_ori.z() = config.Kp_ori_z;

    userParameters.Kp_foot.x() = config.Kp_swing_x;
    userParameters.Kp_foot.y() = config.Kp_swing_y;
    userParameters.Kp_foot.z() = config.Kp_swing_z;

    userParameters.Kd_body.x() = config.Kd_body_x;
    userParameters.Kd_body.y() = config.Kd_body_y;
    userParameters.Kd_body.z() = config.Kd_body_z;

    userParameters.Kd_ori.x() = config.Kd_ori_x;
    userParameters.Kd_ori.y() = config.Kd_ori_y;
    userParameters.Kd_ori.z() = config.Kd_ori_z;

    userParameters.Kd_foot.x() = config.Kd_swing_x;
    userParameters.Kd_foot.y() = config.Kd_swing_y;
    userParameters.Kd_foot.z() = config.Kd_swing_z;

    modifyWbcParameters();

    taskWeight_(0) = config.base_linear_task_weight_sqrt;
    taskWeight_(1) = config.base_ori_task_weight_sqrt;
    taskWeight_(2) = config.leg_tracking_task_weight_sqrt;
    taskWeight_(3) = config.force_tracking_task_weight_sqrt;
    ROS_INFO_STREAM("\033[32m Update the wbc param. \033[0m");
}


vector_t SingleWbcRos::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period, scalar_t time) {
    singleQpTimer_.startTimer();
    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);

    // Task trackingTask = formulateBaseXYZMotionTask() * taskWeight_(0)
    Task trackingTask = formulateBaseAccelTask() * taskWeight_(0)
                        // + formulateBaseAngularMotionTask() * taskWeight_(1)
                        + formulateSwingLegTask() * taskWeight_(2)
                        + formulateContactForceTask(inputDesired) * taskWeight_(3) ;

    Task constraints = formulateFloatingBaseEomTask() + formulateNoContactMotionTask()
                 + formulateTorqueLimitsTask() + formulateFrictionConeTask();
    

    // TrackingQP singQp(trackingTask, costraints);

    int res = qpPtr_->setQpProblem(trackingTask, constraints,isInitRun_);
    isInitRun_ = false;
    if(res != 0){
        ROS_ERROR_STREAM(">>>>Whole-Body Control QP solver failed!<<<<");
        std::cout << "rbdStateMeasured:\n" << rbdStateMeasured.transpose() << "\n";
        std::cout << "stateDesired:\n" << stateDesired.transpose() << "\n";
        std::cout << "inputDesired:\n" << inputDesired.transpose() << "\n";
        std::cout << "mode: " << mode << " period: " << period << " time: " << time << "\n";
        std::cout << "trackingTask\n";
        trackingTask.print();
        std::cout << "constraints\n";
        constraints.print();
    }

    vector_t x_optimal = qpPtr_->getSolutions();
    singleQpTimer_.endTimer();

    return WbcBase::updateCmd(x_optimal); //question 这里的torque的顺序？
                 
// -----------HoQp----------------  
    // Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask()
    //         + formulateNoContactMotionTask() + formulateFrictionConeTask();

    // Task task1 = formulateBaseXYZMotionTask() + formulateBaseAngularMotionTask()
    //              + formulateSwingLegTask() * 100;
    // Task task3 = formulateContactForceTask(inputDesired);

    // HoQp hoQp(task3, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0))); //
    // // HoQp hoQp(task1, std::make_shared<HoQp>(task0)); //
    // vector_t x_optimal = hoQp.getSolutions();
    // singleQpTimer_.endTimer();
    // return WbcBase::updateCmd(x_optimal);
    
}
}
}