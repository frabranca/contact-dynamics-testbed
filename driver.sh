#!/bin/bash

address = "192.168.131.40"

python3 src/controller.py &
sudo ./build/torque_control $address &
sudo ./build/gripper_control $address


