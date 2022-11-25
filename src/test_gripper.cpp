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

    std::cout << gripper_state.width << std::endl;
    std::cout << gripper_state.max_width << std::endl;

    if (gripper_state.width != gripper_state.max_width){
      gripper.homing()
      msg_to_send.homing_done = true;
      std::cout<< "homing done" << std::endl;
    }
    else {
        std::cout << "gripper already in homing position" << std::endl;
    }

    // wait for message to use gripper
    gripper.grasp(gcm_struct.width, gcm_struct.speed, gcm_struct.force);

  } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

  return 0;
}
