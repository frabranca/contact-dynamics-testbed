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

#include "utils/common_functions.cpp"
#include <lcm/lcm-cpp.hpp>
#include "frankalcm/robot_command.hpp"
#include "frankalcm/robot_state.hpp"

// define struct to store received commands from controller
struct command_received{
    std::array<double, 7> tau_received;
    bool loop_closed_received;
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
              rcm_struct.loop_closed_received = msg_received.loop_closed
              for (i=0; i<7; i++){
                rcm_struct.tau_received[i] = msg_received->tau_J_d[i];}
              }
};

int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  lcm::LCM lcm;
  frankalcm::robot_state msg_to_send;
  uint64_t time = 0;  

  Handler handlerObject;
  lcm.subscribe("ROBOT COMMAND", &Handler::handleMessage, &handlerObject);

try {
    // Connect to robot.
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl;
             // << "Press Enter to continue..." << std::endl;
    //std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    
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
    franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

    std::function<franka::Torques(const franka::RobotState&, franka::Duration period)>
        torque_control =
            [&](const franka::RobotState& state, franka::Duration period) -> franka::Torques {
            
        time += period.toMSec();

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

        lcm.publish("ROBOT STATE", &msg_to_send);
        lcm.handle();

        if (rcm_struct.loop_closed == true) {
            std::cout << std::endl << "Finished test" << std::endl;
            return franka::MotionFinished(zero_torques);}
        
        // The following line is only necessary for printing the rate limited torque. As we activated
        // rate limiting for the control loop (activated by default), the torque would anyway be
        // adjusted!
        std::array<double, 7> tau_d_rate_limited = franka::limitRate(franka::kMaxTorqueRate, rcm_struct.tau_received, state.tau_J_d);

    // Send torque command.
    return tau_d_rate_limited;
    };

    // Start real-time control loop.
    robot.control(torque_control);

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
  return 0;
}
