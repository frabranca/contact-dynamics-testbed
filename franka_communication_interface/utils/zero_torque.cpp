#include <cmath>
#include <iostream>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include "utils/common_functions.cpp"

/*
  zero_torque.cpp: used to move the robot by hand. Only zero torque commands are sent.
 */

void message(const char* input){
  std::cout << "zero_torque.cpp " << input << std::endl;
}

int main(int argc, char** argv) {

try {
    // Connect to robot.
    franka::Robot robot("192.168.131.40");
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::array<double, 7> q_goal = {{0.5, 0.0, 0.0, -3 * M_PI_4, -M_PI_2, M_PI_2, M_PI_2}};
    
    MotionGenerator motion_generator(0.5, q_goal);

    robot.control(motion_generator);
    message("Finished moving to initial joint configuration.");
    
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // Define callback for the joint torque control loop.
    franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    std::function<franka::Torques(const franka::RobotState&, franka::Duration period)>
        torque_control =
            [&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
        
        // The following line is only necessary for printing the rate limited torque. As we activated
        // rate limiting for the control loop (activated by default), the torque would anyway be
        // adjusted!
    // Send torque command.
    return zero_torques;
    };

    // Start real-time control loop.
    robot.control(torque_control);

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
  return 0;
}
