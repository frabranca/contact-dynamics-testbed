#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <iostream>
#include <iterator>
#include <mutex>
#include <thread>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>
#include <franka/gripper.h>

#include "utils/common_functions.cpp"
#include <lcm/lcm-cpp.hpp>
#include "frankalcm/gripper_command.hpp"

// define struct to store received commands from controller
struct command_received{
    double width;
    double speed;
    double force;
};

command_received gcm_struct;

// define message handler
class Handler 
{
    public:
        ~Handler() {}
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const frankalcm::gripper_command* msg_received){
              gcm_struct.width = msg_received->width;
              gcm_struct.speed = msg_received->speed;
              gcm_struct.force = msg_received->force;
}
};

int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  lcm::LCM lcm;
  franka::Gripper gripper(argv[1]);
  gripper.homing();

  Handler handlerObject;
  lcm.subscribe("GRIPPER COMMAND", &Handler::handleMessage, &handlerObject);

  try {
    if (gcm_struct.width > 0.0){
      lcm.handle();
      gripper.grasp(gcm_struct.width, gcm_struct.speed, gcm_struct.force);
    }
  
  } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

  return 0;
}
