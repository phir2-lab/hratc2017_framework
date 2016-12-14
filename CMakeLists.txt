cmake_minimum_required(VERSION 2.8.3)
project(hratc2017_framework)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS cmake_modules gazebo_ros message_generation rosconsole roscpp geometry_msgs sensor_msgs nav_msgs metal_detector_msgs tf)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(gazebo REQUIRED)

find_package(Eigen REQUIRED COMPONENTS Dense Cholesky)
include_directories(${Eigen_INCLUDE_DIRS})

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
#add_message_files(
#  DIRECTORY msg
#  FILES
#)

## Generate added messages and services with any dependencies listed here
#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#  nav_msgs
#  sensor_msgs # may not be needed
#)

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
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
  INCLUDE_DIRS
  LIBRARIES 
  CATKIN_DEPENDS gazebo_ros
  DEPENDS gazebo_ros metal_detector_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

### Declare a cpp library
## add_library(fsr_husky_gazebo
##   src/${PROJECT_NAME}/fsr_husky_gazebo.cpp
## )
#add_library(gazebo_mimic_plugin src/gazebo_mimic_plugin/mimic_plugin.cpp)
#add_library(gazebo_gps_plugin src/gazebo_gps_plugin/gazebo_ros_gps.cpp src/UTMConverter/UTMConverter.cpp)

## Declare a cpp executable
#add_executable(arm_controller src/arm_controller.cpp)

### Specify libraries to link a library or executable target against
##target_link_libraries(arm_controller ${catkin_LIBRARIES})
#target_link_libraries(gazebo_mimic_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
#target_link_libraries(gazebo_gps_plugin ${GAZEBO_LIBRARIES})

add_executable(gps_to_odom src/gps_to_odom/gps_to_odom.cpp src/UTMConverter/UTMConverter.cpp)
target_link_libraries(gps_to_odom ${catkin_LIBRARIES})

add_executable(minefield_static_tf_publisher src/minefield_static_tf_publisher/minefield_static_tf_publisher.cpp src/UTMConverter/UTMConverter.cpp)
target_link_libraries(minefield_static_tf_publisher ${catkin_LIBRARIES})

add_executable(judge src/judge/main.cpp src/judge/judge.cpp src/config/config.cpp src/config/robotPose.cpp src/config/trueRobotPose.cpp src/UTMConverter/UTMConverter.cpp)
target_link_libraries(judge ${catkin_LIBRARIES})

add_executable(mineFieldViewer src/minefieldViewer/main.cpp src/minefieldViewer/minefieldviewer.cpp src/config/robotPose.cpp src/config/trueRobotPose.cpp src/config/config.cpp src/UTMConverter/UTMConverter.cpp)
target_link_libraries(mineFieldViewer ${catkin_LIBRARIES})

add_executable(coilSignalSimulator src/coilSignalSimulator/main.cpp src/coilSignalSimulator/CoilSignalSimulator.cpp src/config/robotPose.cpp src/config/trueRobotPose.cpp src/config/config.cpp src/UTMConverter/UTMConverter.cpp)
target_link_libraries(coilSignalSimulator ${catkin_LIBRARIES})

add_executable(convert_coordinates src/UTMConverter/convert_coordinates.cpp src/UTMConverter/UTMConverter.cpp)
target_link_libraries(convert_coordinates ${catkin_LIBRARIES})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(coilSignalSimulator metal_detector_msgs_generate_messages_py)

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
# install(TARGETS fsr_husky_gazebo fsr_husky_gazebo_node
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_fsr_husky_gazebo.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
