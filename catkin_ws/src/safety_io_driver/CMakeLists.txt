cmake_minimum_required(VERSION 3.0.2)
project(safety_io_driver)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Modbus/TCP is implemented with POSIX sockets; no external Modbus library is required.
add_executable(safety_io_node src/safety_io_node.cpp)
target_link_libraries(safety_io_node ${catkin_LIBRARIES})

install(TARGETS safety_io_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(FILES README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
