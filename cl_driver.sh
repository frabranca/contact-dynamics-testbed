#!/bin/bash

./build/gripper_control &
python3 closed_loop/motor_controller.py &
python3 closed_loop/controller.py &
sudo ./build/torque_control




