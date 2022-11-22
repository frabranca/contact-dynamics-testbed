cmake_minimum_required(VERSION 3.20)
project(franka_driver)

set(CMAKE_CXX_STANDARD 17)

find_package(Franka REQUIRED)
find_package(lcm REQUIRED)

include_directories(
  ${CMAKE_SOURCE_DIR}/src
)
  
add_executable(torque_control 
	src/torque_control.cpp)

add_executable(gripper_control
	src/gripper_control.cpp)
	
target_link_libraries(torque_control
	Franka::Franka
	lcm)

target_link_libraries(gripper_control
	Franka::Franka
	lcm)
