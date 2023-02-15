sudo ifconfig lo multicast
sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

cd franka_communication_interface/robot_messages
lcm-gen -xp robot_command.lcm
lcm-gen -xp robot_state.lcm
lcm-gen -xp gripper_command.lcm
cd ../..
cd time_matching/motor_messages
lcm-gen -p motor_command.lcm
cd ../..
cd double_closed_loop/motor_messages
lcm-gen -p motor_command.lcm
cd ../..

echo "LCM bindings built successfully!"
