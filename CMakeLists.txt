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

add_executable(grasp_rod
	src/utils/grasp_rod.cpp)

add_executable(echo_robot_state
	src/utils/echo_robot_state.cpp)
	
add_executable(torque_position_control 
	src/torque_position_control.cpp)

add_executable(position_control
	src/position_control.cpp)

add_executable(pose_motion
	src/utils/generate_cartesian_pose_motion.cpp)

add_executable(time_measure
	src/utils/time_measure.cpp)

add_executable(tm_read_data
	src/time_matching/tm_read_data.cpp)

add_executable(tm_franka
	src/time_matching/tm_franka.cpp)	

add_executable(tm_gripper
	src/time_matching/tm_gripper.cpp)
	
add_executable(tm_vel_control
	src/time_matching/tm_vel_control.cpp)

target_link_libraries(torque_control
	Franka::Franka
	lcm)

target_link_libraries(gripper_control
	Franka::Franka
	lcm)

target_link_libraries(grasp_rod
	Franka::Franka)

target_link_libraries(echo_robot_state
	Franka::Franka)

target_link_libraries(torque_position_control
	Franka::Franka
	lcm)

target_link_libraries(pose_motion
	Franka::Franka
	lcm)
	
target_link_libraries(time_measure
	Franka::Franka
	lcm)
target_link_libraries(position_control
	Franka::Franka
	lcm)

target_link_libraries(tm_read_data
	Franka::Franka
	lcm)
	
target_link_libraries(tm_franka
	Franka::Franka
	lcm)

target_link_libraries(tm_gripper
	Franka::Franka
	lcm)
	
target_link_libraries(tm_vel_control
	Franka::Franka
	lcm)
