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

// define struct to store received commands from controller
struct command_received{
    bool gripper_enable;
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
              gcm_struct.gripper_enable = msg_received->gripper_enable;
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
    franka::GripperState gripper_state = gripper.readOnce();
    gripper.homing();
    message("homing done");

    //msg_to_send.gripper_enabled = true;
    //lcm.publish("GRIPPER STATE", &msg_to_send);
    //message("gripper enabled sent");

    // wait for message to use gripper
    lcm.handle();
    gripper.grasp(0.02, 10.0, 60.0);
    
  } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

  return 0;
}
