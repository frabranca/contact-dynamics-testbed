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

void message(const char* input){
  std::cout << "gripper_control.cpp " << input << std::endl;
}

int main(int argc, char** argv) {
  // Check whether the required arguments were passed.

  lcm::LCM lcm;

  Handler handlerObject;
  lcm.subscribe("GRIPPER COMMAND", &Handler::handleMessage, &handlerObject);

  try {
    // if gripper is fully open send message to controller to say that homing is complete
    franka::Gripper gripper("192.168.131.40");
    gripper.homing();
    message("homing done");

    // wait for message to use gripper
    lcm.handle();
    gripper.grasp(gcm_struct.width, gcm_struct.speed, gcm_struct.force);
    
  } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

  return 0;
}
