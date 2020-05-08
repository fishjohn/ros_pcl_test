cmake_minimum_required(VERSION 2.8.3)
project(ros_pcl)

find_package(catkin REQUIRED COMPONENTS
    pcl_conversions
    pcl_ros
    roscpp
    sensor_msgs
    )

catkin_package(
    INCLUDE_DIRS include
)

include_directories(
    # include
    ${catkin_INCLUDE_DIRS}
)

add_executable(ros_pcl src/ros_pcl.cpp)

target_link_libraries(ros_pcl
    ${catkin_LIBRARIES}
    )
add_compile_options(-Ofast)
