cmake_minimum_required(VERSION 3.10)
project(ocs2_wbc)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)

find_package(catkin REQUIRED
        COMPONENTS
        ocs2_jypro
        )

find_package(raisim CONFIG REQUIRED)
# find_package(raisimOgre CONFIG REQUIRED)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

# generate_dynamic_reconfigure_options(
#         cfg/wbcWeight.cfg
# )

catkin_package(
        INCLUDE_DIRS
        include
        ${raisim_INCLUDE_DIRS}
        LIBRARIES
        ${PROJECT_NAME}
        simple_motion
        CATKIN_DEPENDS
        ocs2_jypro
        DEPENDS
        pinocchio
)

###########
## Build ##
###########



include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${pinocchio_INCLUDE_DIRS}
        ${raisim_INCLUDE_DIRS}
)

link_directories(
        ${pinocchio_LIBRARY_DIRS}
)

add_library(simple_motion
        src/SimpleMotion/SimpleMotion.cpp
        src/SimpleMotion/TerrainEstimator.cpp
        # src/UserParameter.cpp
        )
add_dependencies(simple_motion
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(simple_motion
        ${catkin_LIBRARIES}
        )
target_compile_options(simple_motion PUBLIC ${OCS2_CXX_FLAGS})

add_library(${PROJECT_NAME}
        src/HoQp.cpp
        src/WbcBase.cpp
        # src/HierarchicalWbc.cpp

        src/UserParameter.cpp
        src/TrackingQP.cpp
        src/SingleWbc.cpp
        )
add_dependencies(${PROJECT_NAME}
        ${catkin_EXPORTED_TARGETS}
        )
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${pinocchio_LIBRARIES}
        qpOASES
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

# add_executable(mpc_sim
#         test/mpc_sim.cpp
#         )
# add_dependencies(mpc_sim
#         ${PROJECT_NAME}
#         ${catkin_EXPORTED_TARGETS}
#         )
# target_link_libraries(mpc_sim
#         ${PROJECT_NAME}
#         ${catkin_LIBRARIES}
#         raisim::raisim
#         raisim::raisimOgre
#         )
# target_compile_options(mpc_sim PRIVATE ${OCS2_CXX_FLAGS})
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
install(TARGETS ocs2_wbc
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(TARGETS simple_motion
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


