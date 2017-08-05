cmake_minimum_required(VERSION 2.8.3)
project(claw_com)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
 	${catkin_INCLUDE_DIRS}
)

add_executable(claw_read
  src/claw_read.cpp
  src/uart.cpp 
  src/crc8.cpp
)

target_link_libraries(claw_read
  ${catkin_LIBRARIES}
)

add_executable(claw_write 
  src/claw_write.cpp
  src/uart.cpp 
  src/crc8.cpp
)

target_link_libraries(claw_write
  ${catkin_LIBRARIES}
)
