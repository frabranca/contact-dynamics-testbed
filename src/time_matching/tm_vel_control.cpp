#include <cmath>
#include <iostream>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "utils/common_functions.cpp"
#include <lcm/lcm-cpp.hpp>
#include "robot_messages/frankalcm/robot_command.hpp"
#include "robot_messages/frankalcm/robot_state.hpp"


// define struct to store received commands from controller
struct command_received{
    bool robot_enable;
};

command_received rcm_struct;

// define message handler
class Handler 
{
    public:
        ~Handler() {}
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const frankalcm::robot_command* msg_received){
              int i;
              rcm_struct.robot_enable = msg_received->robot_enable;}

};

void message(const char* input){
  std::cout << "torque_control.cpp " << input << std::endl;
}

int main(int argc, char** argv) {

  lcm::LCM lcm;
  frankalcm::robot_state msg_to_send;

  Handler handlerObject;
  lcm.subscribe("ROBOT COMMAND", &Handler::handleMessage, &handlerObject);

try {
    // Connect to robot.
    franka::Robot robot("192.168.131.40");
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0.0, 0.117397, -0.19942, -2.22072, -1.32267, 1.43232, 1.61111}};
    MotionGenerator motion_generator(0.5, q_goal);

    robot.control(motion_generator);
    message("Finished moving to initial joint configuration.");
    
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, 
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, 
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // Start real-time control loop.
    std::array<double, 7> q_grasp = {{-0.48962, 0.117397, -0.19942, -2.22072, -1.32267, 1.43232, 1.61111}};

    lcm.handle();
    if (rcm_struct.robot_enable == true) {

            double time = 0.0;

            robot.control([&](const franka::RobotState& state,
                             franka::Duration period) -> franka::JointVelocities {
            time += period.toSec();

            franka::JointVelocities output = {{-0.48962*time, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            franka::JointVelocities zero = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            for (int i=0; i<7; i++){
            	msg_to_send.q[i] = state.q[i];
            }
            lcm.publish("ROBOT STATE", &msg_to_send);
            
            if (time >= 1.) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(zero);
      }
      return output;
    });
            
            }

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
  return 0;
}
