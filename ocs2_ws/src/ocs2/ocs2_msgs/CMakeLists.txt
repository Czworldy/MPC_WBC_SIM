cmake_minimum_required(VERSION 3.0.2)
project(ocs2_msgs)

find_package(catkin REQUIRED
  COMPONENTS
    std_msgs
    message_generation
    geometry_msgs
)

add_message_files(
  FILES
    constraint.msg
    mpc_state.msg
    mpc_input.msg
    mode_schedule.msg
    mpc_observation.msg
    mpc_performance_indices.msg
    mpc_target_trajectories.msg
    controller_data.msg
    mpc_flattened_controller.msg
    lagrangian_metrics.msg
    multiplier.msg
    mpc_terrain.msg
    mpc_wbc_conversion.msg
    mpc_wbc_traj.msg
    mpc_target_feetplacement.msg
    FootholdRegion.msg
    FootholdRegionGroup.msg
    RegionForFoot.msg
    RegionSingle.msg
)

add_service_files(
  FILES
    reset.srv
)

generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    std_msgs
    message_runtime
)
