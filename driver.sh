#!/bin/bash

address = 192.168.131.40

python3 src/controller.py &
sleep 1 &
sudo ./build/torque_control $address &
sleep 1 &
sudo ./build/gripper_control $address


