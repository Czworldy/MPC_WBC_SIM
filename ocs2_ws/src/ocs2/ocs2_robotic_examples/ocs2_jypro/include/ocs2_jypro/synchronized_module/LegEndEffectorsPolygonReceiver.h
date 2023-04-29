#pragma once
#include "ocs2_jypro/constraint/Polygon.hpp"
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include "ocs2_jypro/common/Types.h"
#include <ocs2_msgs/RegionForFoot.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_mpc/SystemObservation.h>


#include <ros/ros.h>
#include <mutex>



namespace ocs2 {
namespace legged_robot {
using feet_polygon_array_t = feet_array_t<std::vector<std::vector<vector3_t>>>;
class LegEndEffectorsPolygonReceiver : public SolverSynchronizedModule
{
  public:
    LegEndEffectorsPolygonReceiver(ros::NodeHandle nodeHandle, std::shared_ptr<feet_polygon_array_t> mpcPolygonsPtr, 
                                    std::shared_ptr<feet_array_t<std::vector<vector3_t>>> mpcNominalFeetholdsPtr,
                                    std::shared_ptr<feet_array_t<std::vector<scalar_t>>> mpcSwingHeightPtr,
                                    const std::string& robotName);
    ~LegEndEffectorsPolygonReceiver();

    void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                            const ReferenceManagerInterface& referenceManager) override;
    void postSolverRun(const PrimalSolution& primalSolution) override{};
  private:
    /* data */
    feet_array_t<::ros::Subscriber> mpcPolygonMsgSubscriber_;
    ::ros::Subscriber observationSubscriber_;
    std::mutex receivedPolygonMsgMutex_;
    bool polygonsUpdated_ = false;


    // std::shared_ptr<std::vector<ocs2::Polygon>> legEndeffectorPolygonReceived_;

    feet_polygon_array_t receivedFeetPoints_;
    feet_polygon_array_t transformedFeetPoints_;
    feet_array_t<std::vector<vector3_t>> receivedNominalFeethold_;
    feet_array_t<std::vector<scalar_t>> receivedSwingHeight_;
    feet_array_t<std::vector<vector3_t>> transformedNominalFeethold_;

    std::shared_ptr<feet_polygon_array_t> mpcTransformedPolygonsPtr_;
    std::shared_ptr<feet_array_t<std::vector<vector3_t>>> mpcTransformedNominalFeetholdsPtr_;
    std::shared_ptr<feet_array_t<std::vector<scalar_t>>> mpcSwingHeightPtr_;

    // feet_array_t<std::vector<ocs2::Polygon>>> transformedPolygons_;
    mutable std::mutex latestObservationMutex_;
    SystemObservation latestObservation_;


    void mpcPolygonMsgCallback(const ocs2_msgs::RegionForFoot::ConstPtr& msg);
    // 

};
  
} // namespace legged_robot
} // namespace ocs2

