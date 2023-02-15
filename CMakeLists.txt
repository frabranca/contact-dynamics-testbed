cmake_minimum_required(VERSION 3.20)
project(franka_driver)

set(CMAKE_CXX_STANDARD 17)

find_package(Franka REQUIRED)
find_package(lcm REQUIRED)

include_directories(
  ${CMAKE_SOURCE_DIR}/franka_communication_interface
)

add_executable(torque_control
	franka_communication_interface/torque_control.cpp)
target_link_libraries(torque_control
	Franka::Franka
	lcm)

add_executable(gripper_control
	franka_communication_interface/gripper_control.cpp)
target_link_libraries(gripper_control
	Franka::Franka
	lcm)

add_executable(echo_robot_state
	franka_communication_interface/utils/echo_robot_state.cpp)
target_link_libraries(echo_robot_state
	Franka::Franka)
	
add_executable(zero_torque 
	franka_communication_interface/utils/zero_torque.cpp)
target_link_libraries(zero_torque
	Franka::Franka
	lcm)

add_executable(position_control
	src/position_control.cpp)
target_link_libraries(position_control
	Franka::Franka
	lcm)

add_executable(time_measure
	franka_communication_interface/utils/time_measure.cpp)
target_link_libraries(time_measure
	Franka::Franka
	lcm)
	
add_executable(cartesian_pose
	franka_communication_interface/utils/cartesian_pose.cpp)
target_link_libraries(cartesian_pose
	Franka::Franka)

