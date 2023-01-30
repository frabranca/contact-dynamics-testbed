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
    std::array<double, 7> tau;
    bool robot_moving;
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
              rcm_struct.robot_moving = msg_received->robot_moving;
              for (i=0; i<7; i++){
                rcm_struct.tau[i] = msg_received->tau[i];}
              }
};

void message(const char* input){
  std::cout << "-----" << std::endl;
  std::cout << "torque_control.cpp " << input << std::endl;
  std::cout << "-----" << std::endl;
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
    std::array<double, 7> q_goal = {{0.5, 0.117397, -0.19942, -2.22072, -1.32267, 1.43232, 1.61111}};
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

    // Define callback for the joint torque control loop.
    franka::Torques zero = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    double time = 0.0;
    std::function<franka::Torques(const franka::RobotState&, franka::Duration period)>
        velocity_control =
            [&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
        
        time += period.toSec();
            
        for (int i=0; i<7; i++){
            msg_to_send.q[i] = state.q[i];
            msg_to_send.q_d[i] = state.q_d[i];
            msg_to_send.dq[i] = state.dq[i];
            msg_to_send.dq_d[i] = state.dq_d[i];
            msg_to_send.ddq_d[i] = state.ddq_d[i];
            msg_to_send.tau_J[i] = state.tau_J[i];
            msg_to_send.tau_J_d[i] = state.tau_J_d[i];
            msg_to_send.dtau_J[i] = state.dtau_J[i];
        }

        msg_to_send.robot_enable = true;

        lcm.publish("ROBOT STATE", &msg_to_send);
        
        if (time < 20.){
            lcm.handle();
        }
        
        else{
            return MotionFinished(zero);
        }  

        return rcm_struct.tau;

    };

    // Start real-time control loop.
    // lcm.handle();
    // if (rcm_struct.loop_open_received == true){
    // 	std::cout << "open loop" << std::endl;
    //     robot.control(velocity_control);
    // }
    // Start real-time control loop.
    msg_to_send.robot_enable = true;
    lcm.publish("ROBOT STATE", &msg_to_send);
    robot.control(velocity_control);

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
  return 0;
}
