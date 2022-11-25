#include <cmath>
#include <iostream>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include "utils/common_functions.cpp"
#include <lcm/lcm-cpp.hpp>
#include "robot_messages/frankalcm/gripper_command.hpp"
#include "robot_messages/frankalcm/gripper_state.hpp"

int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    // if gripper is fully open send message to controller to say that homing is complete
    franka::Gripper gripper(argv[1]);
    franka::GripperState gripper_state = gripper.readOnce();

    gripper.homing();
    std::cout<< "homing done" << std::endl;
    

    // wait for message to use gripper
    gripper.grasp(0.02,10.,60.);

  } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

  return 0;
}
