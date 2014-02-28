cmake_minimum_required(VERSION 2.8.3)
project(lms1xx)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)

catkin_package(
  DEPENDS system_lib
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
)

add_executable(LMS1xx_node src/LMS1xx_node.cpp src/LMS1xx.cpp)
target_link_libraries(LMS1xx_node ${catkin_LIBRARIES})

install(TARGETS LMS1xx_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
