cd src
lcm-gen -xp robot_command.lcm
lcm-gen -xp robot_state.lcm
lcm-gen -xp gripper_command.lcm
cd ..
echo "LCM bindings built successfully!"