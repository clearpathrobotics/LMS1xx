cmake_minimum_required(VERSION 2.8.3)
project(LMS1xx)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

include_directories(
  "include"
  ${catkin_INCLUDE_DIRS}
)

add_executable(LMS1xx_node src/LMS1xx_node.cpp src/LMS1xx.cpp)

target_link_libraries(LMS1xx_node
  ${catkin_LIBRARIES}
)
