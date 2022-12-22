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
    MotionGenerator motion_contact_point(0.5, q_grasp);

    robot.setCollisionBehavior(
    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
    {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
    {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}); 

    franka::Duration period;
    running = true;
    double time = 0.0;
    robot.control(motion_generator);

	while (running){
        
        if (time == 0.0){
            robot.control(motion_contact_point);
        };
        if (time == 0.809){
            gripper.grasp(0.01, 10, 60);
        }
        time += period.toSec();
    };

	// Move robot
    robot.control(motion_contact_point);
    gripper.grasp(0.01, 10, 60);

	return 0;
}
