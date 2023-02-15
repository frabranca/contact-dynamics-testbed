cmake_minimum_required(VERSION 3.20)
project(franka_driver)

set(CMAKE_CXX_STANDARD 17)

find_package(Franka REQUIRED)
find_package(lcm REQUIRED)

include_directories(
  ${CMAKE_SOURCE_DIR}/double_closed_loop
)

add_executable(torque_control
	double_closed_loop/torque_control.cpp)
target_link_libraries(torque_control
	Franka::Franka
	lcm)

add_executable(gripper_control
	double_closed_loop/gripper_control.cpp)
target_link_libraries(gripper_control
	Franka::Franka
	lcm)

add_executable(echo_robot_state
	src/utils/echo_robot_state.cpp)
target_link_libraries(echo_robot_state
	Franka::Franka)
	
add_executable(zero_torque 
	time_matching/utils/zero_torque.cpp)
target_link_libraries(zero_torque
	Franka::Franka
	lcm)

add_executable(position_control
	src/position_control.cpp)
target_link_libraries(position_control
	Franka::Franka
	lcm)

add_executable(time_measure
	src/utils/time_measure.cpp)
target_link_libraries(time_measure
	Franka::Franka
	lcm)
	
add_executable(cartesian_pose
	time_matching/utils/cartesian_pose.cpp)
target_link_libraries(cartesian_pose
	Franka::Franka)

