# Contact Dynamics Testbed

![](video.mp4)

## Hardware Instructions
### Franka Robot Arm Hardware Setup
1) Switch on the robot and plug the Ethernet cable directly to the FCI control box. 
2) Switch on the PC and access the real-time kernel from the grub menu ->advanced options->5.9.1-rt20
3) PC password for Francesco Branca user: frankapc22
4) Change the wired network settings of IPv4 to manual. 
    - Set the address to `192.168.131.x`, where x can be any number.
    - Set the netmask to `255.255.255.0`.
5) Open the web browser and enter the address `192.168.131.40`. The Franka desk will open.
6) Unlock the joints from the wireless remote control and from the desk.
7) Pilot Mode: select gripper icon
8) Open the terminal and run the driver.

### Motor Hardware Setup

## Running an Experiment

- Go to this repo's directory. By default it is ~/contact-dynamics-testbed/
- Run `./start_lcm.sh` to setup the network interface for local LCM communication between programs.
- Run `./start_motor.sh` to initiate the motor CAN network.
- Can then directly run `./tm_driver.sh` for the time-matching catch experiment. Take care to see the robot is in good position and has not obstacles. If gripper doesn't work at first try,  trying again usually solves the problem.
- After the capture, reopen the gripper from the button on top. 

## Software Instructions
### Franka Robot Arm Driver
The Franka PANDA research robot is controlled with LCM bindings between C++ and Python. The C++ codes use the franka library (https://frankaemika.github.io/docs/libfranka.html).

To operate the robot libfranka has to be installed, while for real time communication with the robot the controller program on the workstation PC must run with real-time priority under a PREEMPT_RT kernel (https://frankaemika.github.io/docs/installation_linux.html). After installation, the real-time kernel can be found in the grub boot menu. 

### LCM bindings
After installing LCM (https://lcm-proj.github.io/), the bindings are generated using a bash script. To build the bash script the first time before use the following command:
```
chmod 755 start_lcm.sh
ls -l start_lcm.sh
```
To execute the bash file run:
```
./start_lcm.sh
```
This bash file creates the bindings used by the `controller.py` to send commands and receive states from the other components of the testbed. Moreover, it sets up the LCM communication in case the pc is not connected to internet. Note that this bash file has to be run before building and running the executables, since the automatically generated functions are imported and used in the C++ files.

### Build Executable Files
Use following commands to build the executable files `torque_control` and `gripper_control` in the build directory:
```
mkdir build
cd build
cmake ..
make
```
Additionally, the executables `echo_robot_state`, `zero_torque`, `time_measure`, `cartesian_pose` are built.

### Motor Setup
To operate the motor, the bash file `start_motor.sh` has to be run. This initiates the communication via can port with the motor. This command has to be executed until the output says "state UP".

### Running the driver
Two different drivers were made, using different method. The first one is using the time matching method and is run using `tm_driver.sh`, while the second one is using the closed loop method and is run using `cl_driver.sh`. Both of the drivers run the `torque_control.cpp` and `gripper_control.cpp`, which remain unchanged regardless of the method used. The difference between the drivers is in the files `controller.py` and `motor_controller.py`.

```
chmod 755 tm_driver.sh
ls -l tm_driver.sh
./tm_driver.sh
chmod 755 dcl_driver.sh
ls -l cl_driver.sh
./cl_driver.sh
```
