#!/bin/bash

cd build
cmake ..
make
cd ..

python3 src/controller.py &
sleep 1 &
sudo ./build/torque_control 192.168.131.40 &
sleep 1 &
sudo ./build/gripper_control 192.168.131.40


