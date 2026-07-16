cmake_minimum_required(VERSION 3.0.2)
project(bms_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  can_interface
)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs can_interface
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(bms_driver_node
  src/bms_driver_node.cpp
)

target_link_libraries(bms_driver_node
  ${catkin_LIBRARIES}
  Threads::Threads
)

install(TARGETS bms_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
