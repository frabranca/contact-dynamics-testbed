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
    std::array<double, 7> q_goal = {{2.69102, -0.0949951, -2.89166, -2.28845, -1.33746, 1.35165, 1.64393}};
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
    std::array<double, 7> q_grasp = {{2.24287, -0.0773729, -2.89707, -2.27973, -1.33621, 1.38334, 1.63835}};

    lcm.handle();
    if (rcm_struct.robot_enable == true) {

            double time = 0.0;
            robot.control([&](const franka::RobotState&,
                             franka::Duration period) -> franka::CartesianVelocities {
            time += period.toSec();

            franka::CartesianVelocities output = {{-0.44815, 0.0176222, -0.00541, 0.00872, 0.00125, 0.03169, -0.00558 }};
            if (time >= 1.0) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(output);
      }
      return output;
    });
            
            }

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
  return 0;
}
