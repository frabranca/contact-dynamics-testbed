#include <cmath>
#include <iostream>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include "utils/common_functions.cpp"
#include <lcm/lcm-cpp.hpp>
#include "robot_messages/frankalcm/robot_state.hpp"

// define struct to store received commands from controller

int main(int argc, char** argv) {
    lcm::LCM lcm;
    frankalcm::robot_state msg_to_send;

try {
    // Connect to robot.
    franka::Robot robot("192.168.131.40");
    while (true){
        const franka::RobotState& state = robot.readOnce();;
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

    }
    
  } catch (const franka::Exception& ex) {
    std::cerr << ex.what() << std::endl;
  }
  return 0;
}
