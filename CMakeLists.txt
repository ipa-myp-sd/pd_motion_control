cmake_minimum_required(VERSION 2.8.3)
project(pd_motion_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Set catkin run and build packages
SET(CATKIN_RUN_PACKAGES
    actionlib
    actionlib_msgs
    dynamic_reconfigure
    eigen_conversions
    geometry_msgs
    kdl_conversions
    kdl_parser
    roscpp
    sensor_msgs
    std_msgs
    tf
    tf_conversions
    urdf
    visualization_msgs
    shape_msgs
    moveit_msgs
)

# set cakin build package
SET(CATKIN_BUILD_PACKAGES
  ${CATKIN_RUN_PACKAGES}
  cmake_modules
)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_BUILD_PACKAGES})

## System dependencies are found with CMake's conventions
message("Finding Boost library")
find_package(Boost REQUIRED COMPONENTS thread)

message("Finding EIGEN library")
find_package(Eigen REQUIRED)

find_package(Ceres REQUIRED)
message("Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")


find_package(ACADO REQUIRED)
message("Found ACADO version ${ACADO_VERSION}: ${ACADO_INCLUDE_DIRS}")

add_definitions(${EIGEN_DEFINITIONS})

message("Finding KDL library")
find_package(orocos_kdl REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate messages in the 'msg' folder
 add_message_files(
     DIRECTORY
        msg
     FILES
        RobotState.msg
 )

## Generate services in the 'srv' folder
 add_service_files(
     DIRECTORY
        srv
    FILES
        ComputeFK.srv
        ComputeJacobian.srv
 )

## Generate actions in the 'action' folder
# add_action_files(
#     DIRECTORY
#        action
#    FILES
#        Action1.action
#        Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
            actionlib_msgs
            geometry_msgs
            moveit_msgs
            sensor_msgs
            std_msgs
            visualization_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
#generate_dynamic_reconfigure_options(
#  cfg/PDTrajectoryController.cfg
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES pd_motion_control
  CATKIN_DEPENDS ${CATKIN_RUN_PACKAGES}
  DEPENDS ACADO Boost CERES
)

###########
## Build ##
###########

## Specify additional locations of header files
### BUILD ###
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
    ${ACADO_INCLUDE_DIRS}
    #${ACADO_INCLUDE_PACKAGES}
    )

include_directories(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
FIND_LIBRARY(libacado libacado_toolkit_s.so ${ACADO_DIR}/build/lib)

add_executable(kinematics_node src/kinematics.cpp)
add_dependencies(kinematics_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(kinematics_node
    ${catkin_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/pd_motion_control_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############
###install(TARGETS ${PROJECT_NAME} #${PROJECT_NAME}_node
### ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
### LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
### RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
###)

###install(DIRECTORY include/${PROJECT_NAME}/
###    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
###)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY config
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pd_motion_control.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
