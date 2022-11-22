cmake_minimum_required(VERSION 3.20)
project(franka_driver)

set(CMAKE_CXX_STANDARD 17)

find_package(Franka REQUIRED)

include_directories(
  ${CMAKE_SOURCE_DIR}/src
)
  
add_executable(robot_control 
	src/torque_control.cpp
	src/gripper_control.cpp)
	
target_link_libraries(robot_control
	${Franka_LIBRARIES}
	${LCM_NAMESPACE}lcm)
