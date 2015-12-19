cmake_minimum_required(VERSION 2.8.3)
project(lms1xx)

# Build ROS-independent library.
find_package(console_bridge REQUIRED)
include_directories(include ${console_bridge_INCLUDE_DIRS})
add_library(LMS1xx src/LMS1xx.cpp)
target_link_libraries(LMS1xx ${console_bridge_LIBRARIES})

# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
catkin_package(CATKIN_DEPENDS roscpp)

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(LMS1xx_node src/LMS1xx_node.cpp)
target_link_libraries(LMS1xx_node LMS1xx ${catkin_LIBRARIES})


install(TARGETS LMS1xx LMS1xx_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY meshes launch urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(PROGRAMS scripts/find_sick scripts/set_sick_ip
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_buffer test/test_buffer.cpp)
  target_link_libraries(test_buffer ${catkin_LIBRARIES})

  find_package(roslint REQUIRED)
  roslint_cpp()
  #roslint_add_test()

  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/LMS1xx.launch)
endif()

