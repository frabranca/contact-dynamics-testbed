#!/bin/bash

#python3 src/controller.py &
#python3 src/motor_controller.py &
#sudo ./build/torque_control &
#sudo ./build/gripper_control

#./build/gripper_control &
#python3 time_matching/motor_controller.py &
#python3 time_matching/controller.py &
#sudo ./build/torque_control

./build/gripper_control &
python3 double_closed_loop/motor_controller.py &
python3 double_closed_loop/controller.py &
sudo ./build/torque_control




