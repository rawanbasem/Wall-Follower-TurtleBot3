############################
#  wall_follower package   #
############################
cmake_minimum_required(VERSION 3.0.2)
project(wall_follower)

##############################################################################
# 1. Find catkin and declare build/run deps
##############################################################################
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  actionlib_msgs
  message_generation          # <-- enables srv / action generation
)

##############################################################################
# 2. Declare interface files
##############################################################################
## Services
add_service_files(
  DIRECTORY srv               # <package>/srv/FindWall.srv
  FILES   FindWall.srv
)

## Actions
add_action_files(
  DIRECTORY action            # <package>/action/OdomRecord.action
  FILES   OdomRecord.action
)

## Generate the language-specific code for those files
generate_messages(
  DEPENDENCIES
    std_msgs
    geometry_msgs
    actionlib_msgs
)

##############################################################################
# 3. Export to other packages
##############################################################################
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    actionlib_msgs
    message_runtime           # runtime support for generated code
)

##############################################################################
# 4. Include directories (none needed for pure-Python, but keep for completeness)
##############################################################################
include_directories(
  ${catkin_INCLUDE_DIRS}
)