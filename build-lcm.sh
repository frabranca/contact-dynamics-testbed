sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

cd src/robot_messages
lcm-gen -xp robot_command.lcm
lcm-gen -xp robot_state.lcm
lcm-gen -xp gripper_command.lcm
lcm-gen -xp gripper_state.lcm
cd ..
cd motor_messages
lcm-gen -p motor_state.lcm
cd ../..

cd time_matching/robot_messages
lcm-gen -xp robot_command.lcm
lcm-gen -xp robot_state.lcm
lcm-gen -xp gripper_command.lcm
cd ..
cd motor_messages
lcm-gen -p motor_command.lcm
cd ../..

echo "LCM bindings built successfully!"
