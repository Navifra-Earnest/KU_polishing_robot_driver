cmake_minimum_required(VERSION 3.0.2)
project(crevis_io_driver)

add_compile_options(-std=c++14)

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

# Modbus TCP 는 순수 소켓 구현 → 외부 라이브러리 의존 없음
add_executable(crevis_io_node src/crevis_io_node.cpp)
target_link_libraries(crevis_io_node ${catkin_LIBRARIES})

install(TARGETS crevis_io_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
