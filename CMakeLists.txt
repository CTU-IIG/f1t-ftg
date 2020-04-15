cmake_minimum_required(VERSION 2.8.3)
project(follow_the_gap)

set(CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    visualization_msgs
    geometry_msgs
    tf
)

catkin_package(CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    visualization_msgs
    geometry_msgs
    tf
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(follow_the_gap src/main.cpp src/follow_the_gap.cpp src/obstacle.cpp src/gap.cpp src/corner.cpp src/lidar_data.cpp)

target_link_libraries(follow_the_gap
  ${catkin_LIBRARIES}
)
