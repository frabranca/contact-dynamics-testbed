#include <cmath>
#include <iostream>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include "common_functions.cpp"
#include <algorithm>
#include <chrono>
#include<vector>
using namespace std;
using namespace std::chrono;

// For demonstration purpose, we will fill up
// a vector with random integers and then sort
// them using sort function. We fill record
// and print the time required by sort function
int main()
{

    // Connect to robot.
    franka::Robot robot("192.168.131.40");
    franka::Gripper gripper("192.168.131.40");
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::array<double, 7> q_grasp = {{M_PI/2, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};

    MotionGenerator motion_generator(0.5, q_goal);
    robot.control(motion_generator);

    robot.setCollisionBehavior(
    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}); 

	// Get starting timepoint
	auto start_robot = high_resolution_clock::now();

	// Move robot
    MotionGenerator motion_contact_point(0.5, q_grasp);
    robot.control(motion_contact_point);

	// Get ending timepoint
	auto stop_robot = high_resolution_clock::now();
	auto robot_time = duration_cast<microseconds>(stop_robot - start_robot);

	cout << "Robot moving time: "
		<< robot_time.count() << " microseconds" << endl;

    franka::GripperState gripper_state = gripper.readOnce();
    cout << gripper_state.width << ", ";
    
    auto start_gripper = high_resolution_clock::now();
    gripper.grasp(0.01, 10, 60);
    auto stop_gripper = high_resolution_clock::now();
    auto gripper_time = duration_cast<microseconds>(stop_gripper - start_gripper);

	cout << "Gripper closing time: "
		<< gripper_time.count() << " microseconds" << endl;

	return 0;
}
