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
    franka::RobotState robot_state = robot.readOnce();
    
    for (int i=0; i<16; i++){
            std::cout << robot_state.F_T_EE[i] << std::endl;
        }

    }

   catch (franka::Exception const& e) {
    std::cout << e.what() << std::endl;
    return -1;
  }
  return 0;
}
