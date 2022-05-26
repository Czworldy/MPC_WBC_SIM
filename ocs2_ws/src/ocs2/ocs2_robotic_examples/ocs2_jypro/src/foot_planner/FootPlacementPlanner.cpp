#include "ocs2_jypro/foot_planner/FootPlacementPlanner.h"


FootPlacementPlanner::FootPlacementPlanner(const PinocchioInterface& pinocchioInterface,
                                           const CentroidalModelInfo& centroidalModelInfo)
    : pinocchioInterface_(pinocchioInterface),
      endEffectorKinematicsPtr_(new PinocchioEndEffectorKinematics(pinocchioInterface_)),
      centroidalModelInfo_(centroidalModelInfo) {}
{
}