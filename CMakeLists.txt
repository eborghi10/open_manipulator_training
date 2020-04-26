cmake_minimum_required(VERSION 2.8.3)
project(open_manipulator_training)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

find_package(Boost REQUIRED system filesystem date_time thread)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  COMPONENTS
    geometric_shapes
    interactive_markers
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    open_manipulator_gazebo
    open_manipulator_moveit
    tf
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_executable(move_to_ar_box src/move_to_ar_box.cpp)
target_link_libraries(move_to_ar_box ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS move_to_ar_box DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
