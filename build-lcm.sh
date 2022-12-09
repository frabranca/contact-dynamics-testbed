cd src/robot_messages
lcm-gen -xp robot_command.lcm
lcm-gen -xp robot_state.lcm
lcm-gen -xp gripper_command.lcm
lcm-gen -xp gripper_state.lcm
cd ..
cd motor_messages
lcm-gen -p motor_state.lcm
cd ../..
echo "LCM bindings built successfully!"
