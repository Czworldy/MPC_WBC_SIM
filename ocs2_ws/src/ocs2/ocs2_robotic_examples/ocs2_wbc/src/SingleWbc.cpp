//
// Created by czworldy on 2023/4/17.
//

#include "ocs2_wbc/SingleWbc.h"

#include "ocs2_wbc/TrackingQP.h"
#include "ocs2_wbc/HoQp.h"

namespace ocs2{
namespace wbc{
SingleWbc::SingleWbc(const ocs2::PinocchioInterface &pinocchioInterface, ocs2::CentroidalModelInfo info,
                                 const ocs2::PinocchioEndEffectorKinematics &eeKinematics,
                                 const std::string& paramFile)
        : WbcBase(pinocchioInterface, info, eeKinematics, paramFile){
}


vector_t SingleWbc::update(const vector_t& stateDesired, const vector_t& inputDesired, const vector_t& rbdStateMeasured, size_t mode,
                                 scalar_t period, scalar_t time) {

    WbcBase::update(stateDesired, inputDesired, rbdStateMeasured, mode, period, time);

    // Task trackingTask = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask() + formulateBaseXYLinearAccelTask()
    //              + formulateSwingLegTask() * 100 + formulateContactForceTask(inputDesired) * 0.01;
    Task trackingTask = formulateBaseAngularMotionTask() + formulateBaseHeightMotionTask() + formulateSwingLegTask() + formulateBaseXYLinearAccelTask() ;

    // Task equlityConstraints = formulateFloatingBaseEomTask() + formulateNoContactMotionTask();

    // Task inequalityConstraints = formulateTorqueLimitsTask() + formulateFrictionConeTask();
    Task costraints = formulateFloatingBaseEomTask() + formulateNoContactMotionTask()
                 + formulateTorqueLimitsTask() + formulateFrictionConeTask();
    

    TrackingQP singQp(trackingTask, costraints);

    vector_t x_optimal = singQp.getSolutions();
    // std::cout << "x_optimal: " << x_optimal.rows() << std::endl; //rows: 30
    return WbcBase::updateCmd(x_optimal); //question 这里的torque的顺序？
                 
//   
    // Task task0 = formulateFloatingBaseEomTask() + formulateTorqueLimitsTask()
    //         + formulateNoContactMotionTask() + formulateFrictionConeTask();
    // Task task1 = formulateBaseHeightMotionTask() + formulateBaseAngularMotionTask()
    //              + formulateSwingLegTask() * 100;
    // Task task3 = formulateContactForceTask(inputDesired) + formulateBaseXYLinearAccelTask();


    
    
    //     HoQp hoQp(task3, std::make_shared<HoQp>(task1, std::make_shared<HoQp>(task0))); //
    //     // HoQp hoQp(task1, std::make_shared<HoQp>(task0)); //
    //     vector_t x_optimal = hoQp.getSolutions();
    //     return WbcBase::updateCmd(x_optimal);
    
}
}
}