cmake_minimum_required(VERSION 3.0.2)
project(lms1xx)

find_package(Threads REQUIRED)

# Build ROS-independent library.
find_package(console_bridge REQUIRED)
include_directories(include ${console_bridge_INCLUDE_DIRS})

set(LMS1xx_TAG "77fa47e3efa87634bd5f597e0758c4d304a1d3cd")
file(MAKE_DIRECTORY ${CATKIN_DEVEL_PREFIX}/include)
include(ExternalProject)
ExternalProject_Add(lms1xx_src
  GIT_REPOSITORY https://github.com/EasyMile/lms1xx.git
  GIT_TAG ${LMS1xx_TAG}
  CMAKE_ARGS
    -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
)

# Regular catkin package follows.
find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs)
catkin_package(CATKIN_DEPENDS roscpp)
set(CMAKE_CXX_STANDARD 11)

include_directories(include ${catkin_INCLUDE_DIRS} ${CATKIN_DEVEL_PREFIX}/include)
add_executable(LMS1xx_node src/LMS1xx_node.cpp)
add_dependencies(LMS1xx_node lms1xx_src)
target_link_libraries(LMS1xx_node ${catkin_LIBRARIES} ${CATKIN_DEVEL_PREFIX}/lib/liblms1xx.a ${CMAKE_THREAD_LIBS_INIT})

install(TARGETS LMS1xx_node
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

