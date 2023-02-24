# Contact Dynamics Testbed

![](video.mp4)

# Instructions
### Franka Robot Arm Hardware Setup
1) Switch on the robot and plug the Ethernet cable directly to the FCI control box. 
2) Switch on the PC and access the real-time kernel from the grub menu ->advanced options->5.9.1-rt20
3) PC password for Francesco Branca user: frankapc22
4) Change the wired network settings of IPv4 to manual. 
    - Set the address to `192.168.131.x`, where x can be any number.
    - Set the netmask to `255.255.255.0`.
5) Open the web browser and enter the address `192.168.131.40`. The Franka Emika desk will open.
6) Unlock the joints from both the wireless remote control and from the desk by pressing on the unlock button.
7) For the pilot mode select the gripper icon. In this way the gripper can be opened and closed using the buttons on the robot.

### Motor Hardware Setup
1) Connect the motor to the generator and to the CAN-to-USB cable.
2) Attach the USB cable to the PC.
3) Switch on the generator.
4) Unlock the safety switch if it is pressed. 

### Running an Experiment
1) Go to this repo's directory (default it is ~/contact-dynamics-testbed/)

2) Run `./start_lcm.sh` to setup the network interface for local LCM communication between programs.
    - This bash file creates the bindings used by the `controller.py` to send commands and receive states from the
    other components of the testbed. Note that this bash file has to be run before building and running the
    executables, since the automatically generated functions are imported and used in the C++ files.

3) Run `./start_motor.sh` to initiate the motor CAN network.
    - same way as for `start_lcm.sh`, run the extra commands for building the bash script the first time. 

4) To run an experiment using the time matching method  use`./tm_driver.sh` command in the terminal. 
    - Take care to see the robot is in good position and has no obstacles around it. 
    - Keep the safety switch at hand during while the driver is running in case of emergency.
    - If gripper doesn't work at first try,  trying again usually solves the problem.

5) After the capture, reopen the gripper from the button on top.

### Switch off Franka Robot Arm
1) Attach the Ethernet cable to the port below the FCI control box.
2) Keep the network settings the same as before.
3) Open the terminal and write `ssh@administrator 192.168.131.1`
4) Enter the password: `clearpath`.
5) Write `sudo shutdown now`.

# Setting up another PC
If a different PC is used for controlling the testbed, the following instructions have to be followed.
### Franka Robot Arm Driver
The Franka PANDA research robot is controlled with LCM bindings between C++ and Python. The C++ codes use the franka library (https://frankaemika.github.io/docs/libfranka.html).

To operate the robot libfranka has to be installed, while for real time communication with the robot the controller program on the workstation PC must run with real-time priority under a PREEMPT_RT kernel (https://frankaemika.github.io/docs/installation_linux.html). After installation, the real-time kernel can be found in the grub boot menu. 

### LCM bindings
The instructions to install LCM be found on https://lcm-proj.github.io/.

### Build Executable Files
After cloning the repository, use following commands to build the executable files `torque_control` and `gripper_control` in the build directory:
```
cd contact-dynamics-testbed
mkdir build
cd build
cmake ..
make
```
Additionally, the executables `echo_robot_state`, `zero_torque`, `time_measure`, `cartesian_pose` are built.
