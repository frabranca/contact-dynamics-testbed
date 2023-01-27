// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

/**
 * @example echo_robot_state.cpp
 * An example showing how to continuously read the robot state.
 */

int main(int argc, char** argv) {

  try {
    franka::Robot robot("192.168.131.40");    
    std::cout << "Transformation Matrix" << std::endl;
    const franka::RobotState robot_state = robot.readOnce();
    
    for (int i=0; i<4; i++){
            std::cout << robot_state.O_T_EE[i] << ", " <<
            		  robot_state.O_T_EE[i+4] << ", " <<
            		  robot_state.O_T_EE[i+8] << ", " <<
            		  robot_state.O_T_EE[i+12] << " " << std::endl;
        }
    std::cout << " " << std::endl;
    
    std::cout << "Transformation Matrix" << std::endl;
    for (int i=0; i<4; i++){
            std::cout << robot_state.O_T_EE_c[i] << ", " <<
            		  robot_state.O_T_EE_c[i+4] << ", " <<
            		  robot_state.O_T_EE_c[i+8] << ", " <<
            		  robot_state.O_T_EE_c[i+12] << " " << std::endl;
        }
    
    std::cout << " " << std::endl;
    std::cout << "Joint Angles" << std::endl;
    
    for (int i=0; i<7; i++){
    std::cout << robot_state.q[i] << ", ";
    }
    //std::cout << robot_state.O_T_EE_c[14] << std::endl;    
    }

   catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
