//
// Created by czworldy on 2023/4/17.
//

#pragma once

#include "ocs2_wbc/WbcBase.h"
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "ocs2_wbc_ros/wbcWeightConfig.h"
#include <ocs2_core/misc/Benchmark.h>
#include "ocs2_wbc/TrackingQP.h"


namespace ocs2 {
namespace wbc{

class SingleWbcRos : public WbcBase {
public:
    SingleWbcRos(const PinocchioInterface &pinocchioInterface, CentroidalModelInfo info,
                    const PinocchioEndEffectorKinematics &eeKinematics,
                    const std::string& paramFile, ros::NodeHandle& nh);
    ~SingleWbcRos(){
        std::cerr << "\n### SingleWbcRos Benchmarking";
        std::cerr << "\n###   Maximum : " << singleQpTimer_.getMaxIntervalInMilliseconds() << "[ms].";
        std::cerr << "\n###   Average : " << singleQpTimer_.getAverageInMilliseconds() << "[ms].";
    }

    vector_t update(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
                    size_t mode,
                    scalar_t period, scalar_t time) override;
    
    vector_t updateWithContactInfo(const vector_t &stateDesired, const vector_t &inputDesired, const vector_t &rbdStateMeasured,
           const vector_t& forceDesired, size_t mode, scalar_t period, scalar_t time, const ModeSchedule& modeSchedule);

private:
    void dynamicCallback(ocs2_wbc_ros::wbcWeightConfig& config, uint32_t /*level*/);
    std::shared_ptr<dynamic_reconfigure::Server<ocs2_wbc_ros::wbcWeightConfig>> dynamic_srv_{};
    vector_t taskWeight_;
    std::shared_ptr<TrackingQP> qpPtr_;
    bool isInitRun_ = true;

    ros::Publisher pub_, solved_force_pub_, desiredForcePub_;

    benchmark::RepeatedTimer singleQpTimer_;

};
}
}
