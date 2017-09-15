cmake_minimum_required(VERSION 2.8.3)
project(gazebo_pallet_jack)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    std_msgs
    geometry_msgs
    sensor_msgs
    )
find_package(gazebo REQUIRED)
find_package(PkgConfig REQUIRED)
include_directories(${catkin_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})
add_subdirectory(plugins)
catkin_package()

add_executable(pallet_animation plugins/pallet_animation.cc)
target_link_libraries(pallet_animation ${catkin_LIBRARIES})