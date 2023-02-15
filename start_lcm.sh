sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

cd franka_interface/robot_messages
lcm-gen -xp robot_command.lcm
lcm-gen -xp robot_state.lcm
lcm-gen -xp gripper_command.lcm
cd ../..

cd motor_interface/motor_messages
lcm-gen -p motor_command.lcm
lcm-gen -p motor_state.lcm
cd ../..

echo "LCM bindings built successfully!"
