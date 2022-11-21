cmake_minimum_required(VERSION 3.0.2)
project(depth_controller)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs 
  dynamic_reconfigure
)

# add_message_files(
#     FILES
# )

# generate_messages(
#     DEPENDENCIES
#     std_msgs
#)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
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
generate_dynamic_reconfigure_options(
  cfg/PidControl.cfg
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES depth_controller
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  nodes/depth_setpoint.py
  nodes/depth_estimator.py
  nodes/depth_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
