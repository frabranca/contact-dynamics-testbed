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
    std::array<double, 7> tau_received;
    std::array<double, 3> xyz;
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
              rcm_struct.loop_closed_received = msg_received->loop_closed;
              for (i=0; i<3; i++){
                rcm_struct.xyz[i] = msg_received->xyz[i];}
              }
};

void message(const char* input){
  std::cout << "position_control.cpp " << input << std::endl;
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
    std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
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
    std::array<double, 16> initial_pose;
    double time = 0.0;

    std::function<franka::CartesianPose(const franka::RobotState&, franka::Duration period)>
        pose_control = 
            [&](const franka::RobotState& state, franka::Duration period) -> franka::CartesianPose {
            
            time += period.toSec();

            if (time == 0.0) {
                initial_pose = state.O_T_EE_c;}

            // constexpr double kRadius = 0.3;
            // double angle = M_PI / 4 * (1 - std::cos(M_PI / 2.0 * time));
            // double delta_x = kRadius * std::sin(angle);
            // double delta_z = kRadius * (std::cos(angle) - 1);

            lcm.handle();
            double delta_x = rcm_struct.xyz[0];
            double delta_y = rcm_struct.xyz[1];
            double delta_z = rcm_struct.xyz[2]; 
        
            std::array<double, 16> new_pose = initial_pose;
            new_pose[12] += delta_x;
            new_pose[13] += delta_y;
            new_pose[14] += delta_z;
        
            if (time >= 10.0) {
                std::cout << std::endl << "Finished motion, shutting down example" << std::endl;
                return franka::MotionFinished(new_pose);}
      return new_pose;
    };

    // Start real-time control loop.
    msg_to_send.robot_enabled = true;
    lcm.publish("ROBOT STATE", &msg_to_send);
    robot.control(pose_control);

  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
  return 0;
}
