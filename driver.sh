#!/bin/bash

#python3 src/controller.py &
#python3 src/motor_controller.py &
#sudo ./build/torque_control &
#sudo ./build/gripper_control

./build/gripper_control &
python3 time_matching/tm_motor_controller.py &
python3 time_matching/controller.py &
sudo ./build/vel_control






