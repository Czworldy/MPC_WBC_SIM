cmake_minimum_required(VERSION 3.0.2)
project(ocs2_jypro_arm)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  roslib
  cmake_modules
  ocs2_core
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_ddp
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_ros_interfaces
  ocs2_sqp
  tf
  urdf
  kdl_parser
  robot_state_publisher
  ocs2_thirdparty
  X20_ARM
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -std=c++11 -Wfatal-errors -Wl,--no-as-needed")

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    pinocchio
    Boost
)
###########
## Build ##
###########

include_directories(
  include/ocs2_jypro_arm/
  include/ocs2_jypro_arm/BodyPositionEstimator/
  include/ocs2_jypro_arm/taskPriorityControl/
  include/ocs2_jypro_arm/command/
  include/ocs2_jypro_arm/common/
  include/ocs2_jypro_arm/constraint/
  include/ocs2_jypro_arm/cost/
  include/ocs2_jypro_arm/dynamics/
  include/ocs2_jypro_arm/foot_planner/
  include/ocs2_jypro_arm/gait/
  include/ocs2_jypro_arm/initialization/
  include/ocs2_jypro_arm/synchronized_module/
  include/ocs2_jypro_arm/visualization/
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${pinocchio_LIBRARY_DIRS})

# Legged robot interface library
add_library(ocs2_jypro_arm
  src/common/ModelSettings.cpp
  src/common/ManipulatorModelInfo.cpp
  src/dynamics/LeggedRobotWithArmDynamicsAD.cpp
  src/constraint/EndEffectorLinearConstraint.cpp
  src/constraint/EndEffectorConstraint.cpp
  src/constraint/FrictionConeConstraint.cpp
  src/constraint/ZeroForceConstraint.cpp
  src/constraint/NormalVelocityConstraintCppAd.cpp
  src/constraint/ZeroVelocityConstraintCppAd.cpp
  src/cost/LeggedRobotWithArmStateInputQuadraticCost.cpp
  src/cost/QuadraticEndEffectorPositionCostCppAd.cpp
  src/initialization/LeggedRobotWithArmInitializer.cpp
  src/synchronized_module/SwitchedModelReferenceManager.cpp
  src/foot_planner/SplineCpg.cpp
  src/foot_planner/CubicSpline.cpp
  src/foot_planner/SwingTrajectoryPlanner.cpp
  src/gait/Gait.cpp
  src/gait/GaitReceiver.cpp
  src/gait/GaitSchedule.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/LeggedRobotWithArmInterface.cpp
  src/LeggedRobotWithArmPreComputation.cpp
  src/visualization/LeggedRobotWithArmVisualizer.cpp
  
  src/BodyPositionEstimator/BodyPositionEstimator.cpp
  src/BodyPositionEstimator/SystemParameter.cpp

  src/taskPriorityControl/PinocchioInterfaceForArm.cpp
  src/taskPriorityControl/TaskPriorityControl.cpp
  src/taskPriorityControl/Preplanning.cpp
)

add_dependencies(ocs2_jypro_arm
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ocs2_jypro_arm
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
  dl
)
target_compile_options(ocs2_jypro_arm PUBLIC ${pinocchio_CFLAGS_OTHER} -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)


# Mpc node
add_executable(legged_robot_with_arm_mpc
  src/LeggedRobotWithArmMpcNode.cpp
)
add_dependencies(legged_robot_with_arm_mpc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_with_arm_mpc
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(legged_robot_with_arm_mpc PUBLIC ${pinocchio_CFLAGS_OTHER})

# LowerLayerConversions
add_executable(LowerLayerConversion
  src/lowerLayerConversion.cpp
)
add_dependencies(LowerLayerConversion
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(LowerLayerConversion
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(LowerLayerConversion
  PUBLIC 
  ${pinocchio_CFLAGS_OTHER}
)

# MPC_WBC_Conversions
add_executable(MPC_WBC_Conversions
  src/MPC_WBC_Conversions.cpp
)
add_dependencies(MPC_WBC_Conversions
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(MPC_WBC_Conversions
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(MPC_WBC_Conversions 
  PUBLIC 
  ${pinocchio_CFLAGS_OTHER}
)

# MPCProntoConversion
add_executable(MPCProntoConversion
  src/MPCProntoConversion.cpp
)
add_dependencies(MPCProntoConversion
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(MPCProntoConversion
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(MPCProntoConversion PUBLIC ${pinocchio_CFLAGS_OTHER})

# Dummy node
add_executable(legged_robot_with_arm_dummy
  src/LeggedRobotWithArmDummyNode.cpp
)
add_dependencies(legged_robot_with_arm_dummy
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_with_arm_dummy
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_with_arm_dummy PUBLIC ${pinocchio_CFLAGS_OTHER})

# Keyboard Command node for legged robot walking (gait selection and target selection)
add_executable(legged_robot_with_arm_target
  src/LeggedRobotWithArmPoseCommandNode.cpp
)
add_dependencies(legged_robot_with_arm_target
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_with_arm_target
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_with_arm_target PUBLIC ${pinocchio_CFLAGS_OTHER})

# Interactive Command node for legged robot walking (gait selection and target selection)
add_executable(legged_robot_with_arm_target_interactive
  src/LeggedRobotWithArmInteractiveCommandNode.cpp
)
add_dependencies(legged_robot_with_arm_target_interactive
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_with_arm_target_interactive
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_with_arm_target_interactive PUBLIC ${pinocchio_CFLAGS_OTHER})

# Command policy node for legged robot walking (target selection)
add_executable(legged_robot_with_arm_target_upper
  src/LeggedRobotWithArmUpperPolicyNode.cpp
)
add_dependencies(legged_robot_with_arm_target_upper
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_with_arm_target_upper
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_with_arm_target_upper PUBLIC ${pinocchio_CFLAGS_OTHER})


add_executable(legged_robot_with_arm_gait_command
  src/gait/Gait.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/command/LeggedRobotWithArmModeSequenceKeyboard.cpp
  src/LeggedRobotWithArmGaitCommandNode.cpp
)
add_dependencies(legged_robot_with_arm_gait_command
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_with_arm_gait_command
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_with_arm_gait_command PUBLIC ${pinocchio_CFLAGS_OTHER})

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_jypro_arm")
   add_clang_tooling(
     TARGETS
     legged_robot_with_arm_mpc
     legged_robot_with_arm_dummy
     legged_robot_with_arm_target
     legged_robot_with_arm_gait_command
    #  SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(TARGETS ocs2_jypro_arm
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS
  legged_robot_with_arm_mpc
  legged_robot_with_arm_dummy
  legged_robot_with_arm_target
  legged_robot_with_arm_gait_command
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



