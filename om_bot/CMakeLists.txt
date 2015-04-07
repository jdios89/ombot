cmake_minimum_required(VERSION 2.8.3)
project(my_repository)
##THis is the cmakelist for ombot try again
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ps3joy
  roscpp
  rospy
  std_msgs
  tf
  turtlesim
  geometry_msgs
  nav_msgs
  urdf
  sensor_msgs
  message_generation

  move_base_msgs
  actionlib
  costmap_2d
  dynamic_reconfigure

  actionlib_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
generate_dynamic_reconfigure_options(
    cfg/mydynamic.cfg
)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
 add_message_files(
   FILES
    speed_wheel.msg
#   Message1.msg
#   Message2.msg
 )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )
add_service_files(
    FILES
    imuactive.srv
    angularscale.srv
    linearscale.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )
add_action_files(
  DIRECTORY action
  FILES get_close.action
)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#   DEPENDENCIES
#   std_msgs
# )
generate_messages(
    DEPENDENCIES
    std_msgs
    actionlib_msgs
  )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_repository
#  CATKIN_DEPENDS roscpp rospy std_msgs tf
#  DEPENDS system_lib
   CATKIN_DEPENDS message_runtime
   CATKIN_DEPENDS actionlib_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
# add_library(my_repository
#   src/${PROJECT_NAME}/my_repository.cpp
# )
#add_library(simple_layer src/map_plugin/simple_layer.cpp)
add_library(simple_layer_sub src/map_plugin/simple_layer_sub.cpp)
#add_library(grid_layer src/map_plugin/grid_layer.cpp)
## Declare a cpp executable
# add_executable(my_repository_node src/my_repository_node.cpp)
add_executable(pushing src/push_object.cpp)
add_executable(rtf_broadcaster src/rtf_broadcaster.cpp)
add_executable(rtf_listener src/rtf_listener.cpp)
add_executable(rodom_broadcaster src/rodom_broadcaster.cpp)
add_executable(rps3_node src/rps3_node.cpp)
add_executable(rspeed_control src/rspeedcontrol.cpp)
add_executable(rkinematics src/rkinematics.cpp)
add_executable(rodom_broadcastercmd_vel src/rodom_broadcastercmd_vel.cpp)
add_executable(rgoodwheels_s src/rgoodwheels_s.cpp)
add_executable(rgoodwheelsr src/rgoodwheelsr.cpp)
add_executable(ul_range src/ul_range.cpp)
add_executable(imu src/imu_om.cpp)
add_executable(command src/control_transform.cpp)
add_executable(fake_laserr src/fake_laserscan.cpp)
add_executable(get_closse src/close_to_object.cpp)
add_executable(close_action_server src/getclose_action/close_action.cpp)
add_executable(test_action src/test_action.cpp)
#add_executable(push_object src/push_object.cpp)
## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(my_repository_node my_repository_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(my_repository_node
#   ${catkin_LIBRARIES}
# )
add_dependencies(simple_layer_sub 
   ${PROJECT_NAME}_gencfg
  )

target_link_libraries(test_action
   ${catkin_LIBRARIES}
  )
target_link_libraries(close_action_server
   ${catkin_LIBRARIES}
  )
target_link_libraries(pushing
   ${catkin_LIBRARIES}
  )
target_link_libraries(fake_laserr
   ${catkin_LIBRARIES}
  )

target_link_libraries(get_closse
   ${catkin_LIBRARIES}
 )
target_link_libraries(command
   ${catkin_LIBRARIES}
 )

target_link_libraries(rgoodwheels_s
   ${catkin_LIBRARIES}
 )

target_link_libraries(rgoodwheelsr
   ${catkin_LIBRARIES}
 )

target_link_libraries(rps3_node
   ${catkin_LIBRARIES}
 )
target_link_libraries(rspeed_control
   ${catkin_LIBRARIES}
 )
target_link_libraries(rkinematics
   ${catkin_LIBRARIES}
 )
target_link_libraries(rtf_broadcaster 
   ${catkin_LIBRARIES}
 )
target_link_libraries(rtf_listener
   ${catkin_LIBRARIES}
 )
target_link_libraries(rodom_broadcaster
   ${catkin_LIBRARIES}
 )

target_link_libraries(rodom_broadcastercmd_vel
   ${catkin_LIBRARIES}
 )

target_link_libraries(ul_range
   ${catkin_LIBRARIES}
 )

target_link_libraries(imu
   ${catkin_LIBRARIES}
 )

add_dependencies(close_action_server 
   ${my_repositorylib}_EXPORTED_TARGETS
  )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS my_repository my_repository_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_repository.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
