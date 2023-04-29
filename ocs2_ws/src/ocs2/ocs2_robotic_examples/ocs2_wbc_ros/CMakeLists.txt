cmake_minimum_required(VERSION 3.10)
project(ocs2_wbc_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        dynamic_reconfigure
        ocs2_wbc
        ocs2_msgs
        pronto_msgs
        ocs2_legged_robot_raisim)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

generate_dynamic_reconfigure_options(
        cfg/wbcWeight.cfg
)

catkin_package(
        INCLUDE_DIRS
        include
        LIBRARIES
        ${PROJECT_NAME}
        CATKIN_DEPENDS
        roscpp
        ocs2_wbc
        ocs2_msgs
        ocs2_legged_robot_raisim

)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
        src/SingleWbcRos.cpp
        )
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        ${PROJECT_NAME}_gencfg
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        )

target_compile_options(${PROJECT_NAME} PUBLIC ${OCS2_CXX_FLAGS})


# add_executable(angle_test
#         test/angle_test.cpp
#         )
# add_dependencies(angle_test
#         ${PROJECT_NAME}
#         ${catkin_EXPORTED_TARGETS}
#         )
# target_link_libraries(angle_test
#         ${PROJECT_NAME}
#         ${catkin_LIBRARIES}
#         )
# target_compile_options(angle_test PRIVATE ${OCS2_CXX_FLAGS})
find_package(raisim REQUIRED)
find_package(raisimOgre REQUIRED)

add_executable(mpc_sim
        test/mpc_sim.cpp
        )
add_dependencies(mpc_sim
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(mpc_sim
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        raisim::raisim
        raisim::raisimOgre
        )
target_compile_options(mpc_sim PRIVATE ${OCS2_CXX_FLAGS})

add_executable(gazebo_sim
        test/gazebo_sim.cpp
        )
add_dependencies(gazebo_sim
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(gazebo_sim
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        )
target_compile_options(gazebo_sim PRIVATE ${OCS2_CXX_FLAGS})
#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
    message(STATUS "Run clang tooling for target " ${PROJECT_NAME})
    add_clang_tooling(
            TARGETS ${PROJECT_NAME}
            SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
            CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
            CF_WERROR
    )
endif (cmake_clang_tools_FOUND)

#############
## Install ##
#############
install(TARGETS ocs2_wbc_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# install(TARGETS mpc_sim
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


