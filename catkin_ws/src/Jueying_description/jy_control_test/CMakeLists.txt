cmake_minimum_required(VERSION 3.0.2)
project(jy_control_test)

SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall -fopenmp")

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

list(INSERT CMAKE_MODULE_PATH 0 "~/MPC_WBC_sim/catkin_ws/src/Jueying_description/jy_control_test/cmake")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  urdf
  JYPro_msg
  ocs2_msgs
  joint_state_controller
  controller_manager
  gazebo_ros
  #State_Estimate
)

find_package(RBDL  REQUIRED)

find_package(Eigen3 REQUIRED)

find_package(osqp REQUIRED)

find_package(Threads)

find_package(OsqpEigen)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES jy_control_test
  CATKIN_DEPENDS roscpp urdf JYPro_msg joint_state_controller controller_manager gazebo_ros ocs2_msgs
)

include_directories(/usr/local/include/OsqpEigen/)

include_directories(
  include
  include/WBC_CONTROL/dynamics
  include/WBC_CONTROL/wbc/
  include/WBC_CONTROL/wbc_ctrl/TaskSet
  include/WBC_CONTROL/wbc_ctrl
  include/WBC_CONTROL/wbc_ctrl/LocomotionCtrl
  include/FSM_States
  ${catkin_INCLUDE_DIRS}
)

include_directories(${RBDL_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR})

include_directories(/usr/local/include/QuadProg++)

include_directories(/usr/local/include/osqp/)

include_directories(
  include/STATE_ESTIMATE
  include/MAIN

)

include_directories(
  include/PARAMETER
)

include_directories(
  include
  include/Math
)

add_library(${PROJECT_NAME}
                          src/WBC_CONTROL/dynamics/quadruped_dynamics_model.cpp

                          src/WBC_CONTROL/wbc_ctrl/TaskSet/CoMAngularMotion.cpp
                          src/WBC_CONTROL/wbc_ctrl/TaskSet/CoMLinearMotion.cpp
                          src/WBC_CONTROL/wbc_ctrl/TaskSet/ContactForceLimits.cpp
                          src/WBC_CONTROL/wbc_ctrl/TaskSet/ContactForceMin.cpp
                          src/WBC_CONTROL/wbc_ctrl/TaskSet/EoMTask.cpp
                          src/WBC_CONTROL/wbc_ctrl/TaskSet/NoContactMotion.cpp
                          src/WBC_CONTROL/wbc_ctrl/TaskSet/TorqueLimits.cpp
                          src/WBC_CONTROL/wbc_ctrl/TaskSet/SwingLegMotion.cpp

                          src/PARAMETER/UserParameter.cpp
                          src/WBC_CONTROL/wbc/wbc.cpp

                          src/WBC_CONTROL/wbc_ctrl/wbc_ctrl.cpp
                          src/MAIN/simpleMotion.cpp
                          src/STATE_ESTIMATE/TerrainEstimator.cpp
                          # src/PARAMETER/SystemParameter.cpp
                          # src/STATE_ESTIMATE/BodyPositionEst.cpp
)

        
# add_library(MotionPlanner
#                           src/MOTION_PLAN/foot_traj/Gait.cpp
#                           src/MOTION_PLAN/foot_traj/SupportPolygon.cpp
#                           src/MOTION_PLAN/foot_traj/foothold.cpp
#                           src/MOTION_PLAN/foot_traj/FootSwingTrajectory.cpp
#                           src/PARAMETER/UserParameter.cpp
#                           src/PARAMETER/SystemParameter.cpp

#                           src/MOTION_PLAN/CoM_traj/CostTerms/AccelerationMin.cpp
#                           src/MOTION_PLAN/CoM_traj/CostTerms/SoftFinalConstraints.cpp
#                           src/MOTION_PLAN/CoM_traj/CostTerms/DeviationFromPre.cpp
#                           src/MOTION_PLAN/CoM_traj/CostTerms/PathRegularization.cpp
#                           src/MOTION_PLAN/CoM_traj/SearchOptCoefficients.cpp
#                           src/MOTION_PLAN/MotionPlanner.cpp
#                           src/STATE_ESTIMATE/BodyPositionEst.cpp        
# )


# add_executable(motion_plan src/MAIN/motionPlan.cpp)

# target_link_libraries(motion_plan
# ${catkin_LIBRARIES}
# ${PROJECT_NAME}
# ${RBDL_LIBRARY}
# ${EIGEN3_LIBRARY}
# MotionPlanner
# osqp::osqp
# /usr/local/lib/libquadprog.a
# )




add_executable(wbc_pd_trot_mpc src/MAIN/main.cpp)

target_link_libraries(wbc_pd_trot_mpc
 ${catkin_LIBRARIES}
 ${PROJECT_NAME}
 ${RBDL_LIBRARY}
 #${RBDL_URDFReader_LIBRARY}
 ${EIGEN3_LIBRARY}
OsqpEigen::OsqpEigen
/usr/local/lib/libquadprog.a
)

# add_executable(wbc_pd_trot_mpc src/MAIN/wbc_pd_trot_mpc.cpp)

# target_link_libraries(wbc_pd_trot_mpc
#  ${catkin_LIBRARIES}
#  ${PROJECT_NAME}
#  ${RBDL_LIBRARY}
#  #${RBDL_URDFReader_LIBRARY}
#  ${EIGEN3_LIBRARY}
# OsqpEigen::OsqpEigen
# /usr/local/lib/libquadprog.a
# )
