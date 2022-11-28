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
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  lcm::LCM lcm;
  frankalcm::gripper_state msg_to_send;

  Handler handlerObject;
  lcm.subscribe("GRIPPER COMMAND", &Handler::handleMessage, &handlerObject);

  try {
    // if gripper is fully open send message to controller to say that homing is complete
    franka::Gripper gripper(argv[1]);
    franka::GripperState gripper_state = gripper.readOnce();
    gripper.homing();
    message("homing done");

    //msg_to_send.gripper_enabled = true;
    //lcm.publish("GRIPPER STATE", &msg_to_send);
    //message("gripper enabled sent");

    // wait for message to use gripper
    lcm.handle();
    std::cout << gcm_struct.width << ' ';
    std::cout << gcm_struct.speed << ' ';
    std::cout << gcm_struct.force << std::endl;
    gripper.grasp(gcm_struct.width, gcm_struct.speed, gcm_struct.force);
    
  } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

  return 0;
}
