cmake_minimum_required(VERSION 2.8.3)
project(imc_ros_interface)


## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS std_msgs sensor_msgs geometry_msgs nav_msgs roscpp message_runtime message_generation)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system thread)
#add_subdirectory(external)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
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
  Abort.msg
  Chlorophyll.msg
  Depth.msg
  DesiredHeading.msg
  DesiredHeadingRate.msg
  DesiredPitch.msg
  DesiredRoll.msg
  DesiredSpeed.msg
  DesiredZ.msg
  EntityList.msg
  EntityState.msg
  FollowReference.msg
  FollowRefState.msg
  Header.msg
  Heartbeat.msg
  IridiumMsgRx.msg
  IridiumMsgTx.msg
  IridiumTxStatus.msg
  LogBookEntry.msg
  PlanControl.msg
  PlanControlState.msg
  EstimatedState.msg
  ReportControl.msg
  RemoteState.msg
  Salinity.msg
  SonarData.msg
  PlanDB.msg
  PlanSpecification.msg
  PlanDBInformation.msg
  PlanDBState.msg
  PlanManeuver.msg
  Maneuver.msg
  Reference.msg
  Sms.msg
  Temperature.msg
  VehicleMedium.msg
  VehicleState.msg
  Waypoint.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES imc_ros_interface
 CATKIN_DEPENDS message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

# THINGS TO DO WHEN ADDING NEW MESSAGES (0/3)

# 1/3
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  external/imc-cxx
  external/md5
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library( md5 external/md5/md5.cpp )
add_library( imc_factory external/imc-cxx/IMC/Base/Factory.cpp )
add_library( imc_handle src/imc_handle.cpp )

add_library( ros_to_imc_gpsfix src/ros_to_imc/GpsFix.cpp )
add_library( ros_to_imc_goto src/ros_to_imc/Goto.cpp )
add_library( ros_to_imc_plandb src/ros_to_imc/PlanDB.cpp )
add_library( ros_to_imc_gpsnavdata src/ros_to_imc/GpsNavData.cpp )
add_library( ros_to_imc_remotestate src/ros_to_imc/RemoteState.cpp )
add_library( ros_to_imc_sonardata src/ros_to_imc/SonarData.cpp )
add_library( ros_to_imc_DesiredHeading src/ros_to_imc/DesiredHeading.cpp )
add_library( ros_to_imc_DesiredHeadingRate src/ros_to_imc/DesiredHeadingRate.cpp )
add_library( ros_to_imc_DesiredPitch src/ros_to_imc/DesiredPitch.cpp )
add_library( ros_to_imc_DesiredRoll src/ros_to_imc/DesiredRoll.cpp )
add_library( ros_to_imc_EntityList src/ros_to_imc/EntityList.cpp)
add_library( ros_to_imc_FollowReference src/ros_to_imc/FollowReference.cpp)
add_library( ros_to_imc_Reference src/ros_to_imc/Reference.cpp)
add_library( ros_to_imc_LogBookEntry src/ros_to_imc/LogBookEntry.cpp)
add_library( ros_to_imc_Sms src/ros_to_imc/Sms.cpp)
add_library( ros_to_imc_IridiumMsgTx src/ros_to_imc/IridiumMsgTx.cpp)
add_library( ros_to_imc_ReportControl src/ros_to_imc/ReportControl.cpp)

add_library(imc_to_ros_Heartbeat src/imc_to_ros/Heartbeat.cpp )
add_library(imc_to_ros_goto src/imc_to_ros/Goto.cpp )
add_library(imc_to_ros_plandb src/imc_to_ros/PlanDB.cpp )
add_library(imc_to_ros_EntityState src/imc_to_ros/EntityState.cpp)
add_library(imc_to_ros_Temperature src/imc_to_ros/Temperature.cpp)
add_library(imc_to_ros_Chlorophyll src/imc_to_ros/Chlorophyll.cpp)
add_library(imc_to_ros_Salinity src/imc_to_ros/Salinity.cpp)
add_library(imc_to_ros_Plancontrol src/imc_to_ros/PlanControl.cpp )
add_library(imc_to_ros_Depth src/imc_to_ros/Depth.cpp)
add_library(imc_to_ros_VehicleState src/imc_to_ros/VehicleState.cpp)
add_library(imc_to_ros_EstimatedState src/imc_to_ros/EstimatedState.cpp)
add_library(imc_to_ros_EntityList src/imc_to_ros/EntityList.cpp)
add_library(imc_to_ros_VehicleMedium src/imc_to_ros/VehicleMedium.cpp)
add_library(imc_to_ros_PlanControlState src/imc_to_ros/PlanControlState.cpp)
add_library(imc_to_ros_FollowRefState src/imc_to_ros/FollowRefState.cpp)
add_library(imc_to_ros_Abort src/imc_to_ros/Abort.cpp)
add_library(imc_to_ros_IridiumTxStatus src/imc_to_ros/IridiumTxStatus.cpp)
add_library(imc_to_ros_IridiumMsgRx src/imc_to_ros/IridiumMsgRx.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(imc_to_ros_plandb imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_Plancontrol imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_EntityState imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_Temperature imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_Chlorophyll imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_Salinity imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_Depth imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_VehicleState imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_EstimatedState imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_Heartbeat imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_EntityList imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_VehicleMedium imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_PlanControlState imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_FollowRefState imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_Abort imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_IridiumTxStatus imc_ros_interface_generate_messages_cpp)
add_dependencies(imc_to_ros_IridiumMsgRx imc_ros_interface_generate_messages_cpp)

add_dependencies(ros_to_imc_plandb imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_sonardata imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_remotestate imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_DesiredHeading imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_DesiredHeadingRate imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_DesiredPitch imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_DesiredRoll imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_EntityList imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_FollowReference imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_Reference imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_LogBookEntry imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_Sms imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_IridiumMsgTx imc_ros_interface_generate_messages_cpp)
add_dependencies(ros_to_imc_ReportControl imc_ros_interface_generate_messages_cpp)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/imc_ros_interface_node.cpp)
add_executable(bridge_node src/bridge_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

# 2/3
## Specify libraries to link a library or executable target against
target_link_libraries(md5 ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_Heartbeat ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_goto ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_plandb ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_Plancontrol ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_EntityState ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_Temperature ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_Chlorophyll ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_Salinity ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_Depth ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_VehicleState ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_EntityList ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_VehicleMedium ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_PlanControlState ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_FollowRefState ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_Abort ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_IridiumTxStatus ${catkin_LIBRARIES})
target_link_libraries(imc_to_ros_IridiumMsgRx ${catkin_LIBRARIES})


target_link_libraries(ros_to_imc_gpsfix ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_goto ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_gpsnavdata ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_remotestate ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_sonardata ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_DesiredHeading ${catkin_LIBRARIES} )
target_link_libraries(ros_to_imc_DesiredHeadingRate ${catkin_LIBRARIES} )
target_link_libraries(ros_to_imc_DesiredPitch ${catkin_LIBRARIES} )
target_link_libraries(ros_to_imc_DesiredRoll ${catkin_LIBRARIES} )
target_link_libraries(ros_to_imc_plandb ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_EntityList ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_FollowReference ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_Reference ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_LogBookEntry ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_Sms ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_IridiumMsgTx ${catkin_LIBRARIES})
target_link_libraries(ros_to_imc_ReportControl ${catkin_LIBRARIES})

target_link_libraries(imc_handle
  imc_factory
  ${Boost_LIBRARIES}
)

# 3/3
target_link_libraries(bridge_node
  imc_to_ros_Heartbeat
  imc_to_ros_goto
  imc_to_ros_plandb
  imc_to_ros_Plancontrol
  imc_to_ros_EntityState
  imc_to_ros_Temperature
  imc_to_ros_Chlorophyll
  imc_to_ros_Salinity
  imc_to_ros_Depth
  imc_to_ros_VehicleState
  imc_to_ros_EstimatedState
  imc_to_ros_EntityList
  imc_to_ros_VehicleMedium
  imc_to_ros_PlanControlState
  imc_to_ros_FollowRefState
  imc_to_ros_Abort
  imc_to_ros_IridiumTxStatus
  imc_to_ros_IridiumMsgRx
  ros_to_imc_plandb
  ros_to_imc_gpsfix
  ros_to_imc_goto
  ros_to_imc_gpsnavdata
  ros_to_imc_remotestate
  ros_to_imc_sonardata
  ros_to_imc_DesiredHeading
  ros_to_imc_DesiredHeadingRate
  ros_to_imc_DesiredPitch
  ros_to_imc_DesiredRoll
  ros_to_imc_EntityList
  ros_to_imc_FollowReference
  ros_to_imc_Reference
  ros_to_imc_LogBookEntry
  ros_to_imc_Sms
  ros_to_imc_IridiumMsgTx
  ros_to_imc_ReportControl
  imc_handle
  md5
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
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
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_imc_ros_interface.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
