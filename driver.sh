#!/bin/bash

ip = 192.168.131.40

cd build
cmake ..
make
cd ..

python3 src/controller.py &
sleep 1 &
sudo ./build/torque_control $ip