#include <cmath>
#include <iostream>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include "utils/common_functions.cpp"

// test to grasp metal rod with the 3D printed gripper clamp
// the grasping position can be found by moving the franka by hand and running the code echo_robot_state.cpp

int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  try {
    // if gripper is fully open send message to controller to say that homing is complete
    franka::Robot robot(argv[1]);
    franka:Gripper gripper(argv[1]);

    gripper.homing();
    // insert grasping position
    std::array<double, 7> q_grasp = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_grasp);
    gripper.grasp(0.02, 10, 60);

  } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

  return 0;
}
