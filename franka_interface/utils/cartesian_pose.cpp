#include <cmath>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include "common_functions.cpp"

/*
 
 cartesian_pose.cpp: 
  - sends cartesian pose command to franka robot.
  - deos not use lcm communication with python file.

*/

int main(int argc, char** argv) {
  try {
    franka::Robot robot("192.168.131.40");

    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;

    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    std::array<double, 16> initial_pose;
    double time = 0.0;
    robot.control([&time, &initial_pose](const franka::RobotState& robot_state,
                                         franka::Duration period) -> franka::CartesianPose {
      time += period.toSec();

      if (time == 0.0) {
        initial_pose = robot_state.O_T_EE_c;
      }

      std::cout << time << ",";
      std::cout << robot_state.O_T_EE_c[12] << ",";
      std::cout << robot_state.O_T_EE_c[13] << ",";
      std::cout << robot_state.O_T_EE_c[14] << std::endl;      

      constexpr double kRadius = 0.3;
      double angle = M_PI / 4 * (1 - std::cos(M_PI / 2.0 * time));
      double delta_x = kRadius * std::sin(angle);
      double delta_z = kRadius * (std::cos(angle) - 1);

      std::array<double, 16> new_pose = initial_pose;
      new_pose[12] += delta_x;
      new_pose[14] += delta_z;

      if (time >= 10.0) {
        std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
        return franka::MotionFinished(new_pose);
      }
      return new_pose;
    });
  } catch (const franka::Exception& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}
