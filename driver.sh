#!/bin/bash

python3 src/controller.py &
python3 src/motor_controller.py &
sudo ./build/torque_control 192.168.131.40 &
sudo ./build/gripper_control 192.168.131.40





