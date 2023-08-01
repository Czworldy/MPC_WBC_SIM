cmake_minimum_required(VERSION 3.0.2)
project(ocs2_jypro)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)

set(CATKIN_PACKAGE_DEPENDENCIES
  roslib
  pybind11_catkin
  cmake_modules
  ocs2_core
  ocs2_mpc
  ocs2_robotic_tools
  ocs2_ddp
  ocs2_pinocchio_interface
  ocs2_centroidal_model
  ocs2_ros_interfaces
  ocs2_python_interface
  ocs2_sqp
  ocs2_self_collision
  ocs2_self_collision_visualization
  tf
  urdf
  kdl_parser
  robot_state_publisher
  pronto_msgs
  # teb_local_planner
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
  # teb_local_planner
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)


# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread -fopenmp -Wfatal-errors -Wl,--no-as-needed")


###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    include/ocs2_jypro/BodyPositionEstimator/
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

# Resolve for the package path at compile time.
configure_file (
  "${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/package_path.h.in"
  "${PROJECT_BINARY_DIR}/include/${PROJECT_NAME}/package_path.h" @ONLY
)

set(FLAGS
  ${OCS2_CXX_FLAGS}
  ${pinocchio_CFLAGS_OTHER}
  -Wno-ignored-attributes
  -Wno-invalid-partial-specialization   # to silence warning with unsupported Eigen Tensor
  -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR
  -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR
)

include_directories(
  include
  test/include
  ${pinocchio_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
link_directories(${pinocchio_LIBRARY_DIRS})

# Legged robot interface library
add_library(ocs2_jypro
  src/common/ModelSettings.cpp
  src/dynamics/LeggedRobotDynamicsAD.cpp
  src/constraint/EndEffectorLinearConstraint.cpp
  src/constraint/FrictionConeConstraint.cpp
  src/constraint/ZeroForceConstraint.cpp
  src/constraint/NormalVelocityConstraintCppAd.cpp
  src/constraint/ZeroVelocityConstraintCppAd.cpp
  # src/constraint/StateOnlyFootPlacementConstraintCppAD.cpp
  src/constraint/StateOnlyFootPlacementConstraint.cpp
  # src/constraint/CBFFootPlacementConstraintCppAD.cpp
  src/constraint/Polygon.cpp
  src/cost/LeggedRobotStateInputQuadraticCost.cpp
  src/cost/LeggedRobotEndEffectorCost.cpp
  src/cost/LeggedRobotQuadraticStateInputCost.cpp
  src/initialization/LeggedRobotInitializer.cpp
  # src/synchronized_module/LeggedRobotReferenceManager.cpp
  src/synchronized_module/SwitchedModelReferenceManager.cpp
  # src/synchronized_module/LeggedRobotRosReferenceManager.cpp
  src/synchronized_module/TerrainReceiver.cpp
  src/synchronized_module/TerrainPythonInterface.cpp
  src/foot_planner/SplineCpg.cpp
  src/foot_planner/CubicSpline.cpp
  src/foot_planner/QuinticSpline.cpp
  src/foot_planner/TwoQuinticSplineCpg.cpp
  src/foot_planner/MultiSplineCpg.cpp
  src/foot_planner/SixthOrderSpline.cpp
  src/foot_planner/SwingTrajectoryPlanner.cpp
  src/foot_planner/FootPlacementPlanner.cpp
  src/foot_planner/FootConstraintsPlanner.cpp
  src/foot_planner/LeggedIKSolver.cpp
  src/foot_planner/MinimumJerkSeventhOrderSpline.cpp
  src/gait/Gait.cpp
  src/gait/GaitReceiver.cpp
  src/gait/GaitPythonInterface.cpp
  src/gait/GaitSchedule.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/LeggedRobotInterface.cpp
  src/LeggedRobotPreComputation.cpp
  src/LeggedRobotPinocchioMapping.cpp
  src/visualization/LeggedRobotVisualizer.cpp
  src/visualization/FootPlacementVisualizer.cpp
  src/synchronized_module/LegEndEffectorsPolygonReceiver.cpp

)

add_dependencies(ocs2_jypro
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(ocs2_jypro
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${pinocchio_LIBRARIES}
  dl
  qpOASES
)
target_compile_options(ocs2_jypro PUBLIC PUBLIC ${pinocchio_CFLAGS_OTHER} -DPINOCCHIO_URDFDOM_TYPEDEF_SHARED_PTR -DPINOCCHIO_URDFDOM_USE_STD_SHARED_PTR)

# Mpc node
add_executable(legged_robot_mpc
  src/LeggedRobotMpcNode.cpp
)
add_dependencies(legged_robot_mpc
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_mpc
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
target_compile_options(legged_robot_mpc PUBLIC ${pinocchio_CFLAGS_OTHER})

# MPC_WBC_Conversions
# add_executable(MPC_WBC_Conversions
#   src/MPC_WBC_Conversions.cpp
# )
# add_dependencies(MPC_WBC_Conversions
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(MPC_WBC_Conversions
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# target_compile_options(MPC_WBC_Conversions PUBLIC ${pinocchio_CFLAGS_OTHER})

# MPCProntoConversion
# add_executable(MPCProntoConversion
#   src/MPCProntoConversion.cpp
# )
# add_dependencies(MPCProntoConversion
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(MPCProntoConversion
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# target_compile_options(MPCProntoConversion PUBLIC ${pinocchio_CFLAGS_OTHER})

# MPCProntoConversionUDP
# add_executable(MPCProntoConversionUDP
#   src/MPCProntoConversionUDP.cpp
# )
# add_dependencies(MPCProntoConversionUDP
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(MPCProntoConversionUDP
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# target_compile_options(MPCProntoConversionUDP PUBLIC ${pinocchio_CFLAGS_OTHER})

# MPC_WBC_ConversionsUDP
# add_executable(MPC_WBC_ConversionsUDP
#   src/MPC_WBC_ConversionsUDP.cpp
# )
# add_dependencies(MPC_WBC_ConversionsUDP
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(MPC_WBC_ConversionsUDP
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# target_compile_options(MPC_WBC_ConversionsUDP PUBLIC ${pinocchio_CFLAGS_OTHER})

# Dummy node
add_executable(legged_robot_dummy
  src/LeggedRobotDummyNode.cpp
)
add_dependencies(legged_robot_dummy
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_dummy
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_dummy PUBLIC ${pinocchio_CFLAGS_OTHER})

# Keyboard Command node for legged robot walking (gait selection and target selection)
add_executable(legged_robot_target
  src/LeggedRobotPoseCommandNode.cpp
)
add_dependencies(legged_robot_target
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_target
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_target PUBLIC ${pinocchio_CFLAGS_OTHER})

# Joy Command node for legged robot walking (gait selection and target selection)
add_executable(legged_robot_target_joy
  src/LeggedRobotPoseCommandJoyNode.cpp
  src/command/TargetTrajectoriesJoyPublisher.cpp
)
add_dependencies(legged_robot_target_joy
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_target_joy
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_target_joy PUBLIC ${pinocchio_CFLAGS_OTHER})

# Traj Command node for legged robot walking (gait selection and target selection)
add_executable(legged_robot_target_traj
  src/LeggedRobotPoseCommandTrajNode.cpp
  src/command/TargetTrajectoriesPublisher.cpp
)
add_dependencies(legged_robot_target_traj
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_target_traj
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_target_traj PUBLIC ${pinocchio_CFLAGS_OTHER})


add_executable(legged_robot_gait_command
  src/gait/Gait.cpp
  src/gait/ModeSequenceTemplate.cpp
  src/command/LeggedRobotModeSequenceKeyboard.cpp
  src/LeggedRobotGaitCommandNode.cpp
)
add_dependencies(legged_robot_gait_command
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(legged_robot_gait_command
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
target_compile_options(legged_robot_gait_command PUBLIC ${pinocchio_CFLAGS_OTHER})

# # python bindings
# pybind11_add_module(LeggedRobotPyBindings SHARED
#   src/pyBindModule.cpp
# )
# add_dependencies(LeggedRobotPyBindings
#   ${PROJECT_NAME}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(LeggedRobotPyBindings PRIVATE
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )
# set_target_properties(LeggedRobotPyBindings
#     PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION}
# )

# # python bindings
# pybind11_add_module(Polygon SHARED
#   script/pyPolygon.cpp
# )
# add_dependencies(Polygon
#   ${PROJECT_NAME}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(Polygon PRIVATE
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
# )
# set_target_properties(Polygon
#     PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/script
# )

catkin_python_setup()

#test publisher
# add_executable(TestFeetPlacementPublisher
#   src/TestFeetPlacementPublisher.cpp
# )
# add_dependencies(TestFeetPlacementPublisher
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )
# target_link_libraries(TestFeetPlacementPublisher
#   ${PROJECT_NAME}
#   ${catkin_LIBRARIES}
#   ${Boost_LIBRARIES}
# )

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if(cmake_clang_tools_FOUND)
   message(STATUS "Run clang tooling for target ocs2_jypro")
   add_clang_tooling(
     TARGETS
     legged_robot_mpc
     legged_robot_dummy
     legged_robot_target
     legged_robot_gait_command
     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/test
     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
     CF_WERROR
)
endif(cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(TARGETS ocs2_jypro
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(TARGETS
  legged_robot_mpc
  legged_robot_dummy
  legged_robot_target
  legged_robot_gait_command
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
catkin_add_gtest(${PROJECT_NAME}_test
  test/AnymalFactoryFunctions.cpp
  test/constraint/testEndEffectorLinearConstraint.cpp
  test/constraint/testFrictionConeConstraint.cpp
  test/constraint/testZeroForceConstraint.cpp
)
target_link_libraries(${PROJECT_NAME}_test
  gtest_main
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)
