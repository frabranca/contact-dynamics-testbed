#!/bin/bash

./build/gripper_control &
python3 time_matching/motor_controller.py &
python3 time_matching/controller.py &
sudo ./build/torque_control

