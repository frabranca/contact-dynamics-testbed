#include <cmath>
#include <iostream>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include "common_functions.cpp"

/* grasp_rod.cpp: used to test the 3D printer gripper. 
*/

int main(int argc, char** argv) {
  // Check whether the required arguments were passed.

  try {
    // if gripper is fully open send message to controller to say that homing is complete
    franka::Robot robot("192.168.131.40");
    franka::Gripper gripper("192.168.131.40");

    gripper.homing();
    // insert grasping position
    //std::array<double, 7> q_grasp = {{0, -M_PI_4, 0, -3 * M_PI_4, -M_PI_2, M_PI_2, 3*M_PI_4}};
    std::array<double, 7> q_grasp = {{0, -M_PI_4, 0, -3 * M_PI_4, -M_PI_2, M_PI_2, 3*M_PI_4}};
    
    MotionGenerator motion_generator(0.5, q_grasp);
    robot.control(motion_generator);
    gripper.grasp(0.02, 10, 60);

  } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

  return 0;
}
