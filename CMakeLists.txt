cmake_minimum_required(VERSION 2.8.3)
project(cassie_common_toolbox)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp 
  rospy 
  genmsg 
  std_msgs 
  geometry_msgs 
  ros_utilities 
  eigen_utilities
  control_utilities 
  realtime_utilities
  control_eigen_utilities
)

################################################
## Declare ROS messages, services and actions ##
################################################
# Specify custom messages
add_message_files(DIRECTORY msg FILES cassie_control_msg.msg cassie_estimation_msg.msg cassie_proprioception_msg.msg)

# Generate added messages
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy ros_utilities eigen_utilities control_utilities realtime_utilities control_eigen_utilities
  DEPENDS
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/smoothing.cpp
    src/bezier_tools.cpp
    src/PhaseVariable.cpp
    src/motion_transition.cpp
    src/linear_interpolation.cpp
    src/geometry.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
