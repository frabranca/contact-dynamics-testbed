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
    bool start_gripper;
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
              gcm_struct.start_gripper = msg_received->start_gripper;
}
};

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
    gripper.homing()
    //msg_to_send.homing_done = true;
    //lcm.publish("GRIPPER STATE", &msg_to_send);
    std::cout<< "homing done" << std::endl;

    // wait for message to use gripper
    lcm.handle();
    if (gcm_struct.start_gripper){
      lcm.handle()
      gripper.grasp(gcm_struct.width, gcm_struct.speed, gcm_struct.force);
    }
    
  } catch (franka::Exception const& e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

  return 0;
}
