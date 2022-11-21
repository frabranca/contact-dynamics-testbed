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

#include "examples_common.cpp"
#include <lcm/lcm-cpp.hpp>
#include "exlcm/command.hpp"
#include "exlcm/state.hpp"


// compile with: g++ -o torque_control torque_control.cpp -lfranka -lpthread -llcm

struct command_received{
    std::array<double, 7> tau_received;
};

command_received Command;

class Handler 
{
    public:
        ~Handler() {}
        void handleMessage(const lcm::ReceiveBuffer* rbuf,
                const std::string& chan, 
                const exlcm::command* msg_received){
              int i;
              for (i=0; i<7; i++)
                Command.tau_received[i] = msg_received->tau_J_d[i];
                }
};

int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  std::atomic_bool running{true};
  lcm::LCM lcm;
  exlcm::state msg_to_send;

  Handler handlerObject;
  lcm.subscribe("COMMAND", &Handler::handleMessage, &handlerObject);


try {
    // Connect to robot.
    franka::Robot robot(argv[1]);
    franka::Gripper gripper(argv[1]);
    setDefaultBehavior(robot);

    // First move the robot to a suitable joint configuration
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
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
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        torque_control =
            [&](const franka::RobotState& state, franka::Duration /*period*/) -> franka::Torques {
        
        Handler handlerObject;
        lcm.subscribe("COMMAND", &Handler::handleMessage, &handlerObject);
        
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
        
        msg_to_send.width = gripper_state.width;
        msg_to_send.max_width = gripper_state.max_width;
        msg_to_send.is_grasped = gripper_state.is_grasped;

        lcm.publish("STATE", &msg_to_send);

        lcm.handle();
        // The following line is only necessary for printing the rate limited torque. As we activated
        // rate limiting for the control loop (activated by default), the torque would anyway be
        // adjusted!
        std::array<double, 7> tau_d_rate_limited = franka::limitRate(franka::kMaxTorqueRate, Command.tau_received, state.tau_J_d);

    // Send torque command.
    return tau_d_rate_limited;
    };


    // Start real-time control loop.
    // gripper.grasp(0.02, 10.0, 60);
    // std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));
    robot.control(torque_control);

    
    // double grasping_width = 0.02;
    // gripper.grasp(grasping_width, 0.1, 60);
    // std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(3000));

  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }
  return 0;
}
