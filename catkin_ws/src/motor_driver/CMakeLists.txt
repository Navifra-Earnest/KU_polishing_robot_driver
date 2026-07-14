cmake_minimum_required(VERSION 3.0.2)
project(motor_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(motor_driver_node
  src/motor_driver_node.cpp
  src/can_interface.cpp
  src/motor_controller.cpp
)

target_link_libraries(motor_driver_node
  ${catkin_LIBRARIES}
  Threads::Threads
)

add_executable(base_controller_node
  src/base_controller_node.cpp
)

target_link_libraries(base_controller_node
  ${catkin_LIBRARIES}
)

add_executable(bms_driver_node
  src/bms_driver_node.cpp
  src/can_interface.cpp
)

target_link_libraries(bms_driver_node
  ${catkin_LIBRARIES}
  Threads::Threads
)

install(TARGETS motor_driver_node base_controller_node bms_driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS scripts/teleop_keyboard.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
