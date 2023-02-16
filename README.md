## Contact Dynamics Test Bed

# Franka Robot Arm Driver
The Franka PANDA research robot is controlled with LCM bindings between C++ and Python. The C++ codes use the franka library (https://frankaemika.github.io/docs/libfranka.html).

To operate the robot libfranka has to be installed, while for real time communication with the robot the controller program on the workstation PC must run with real-time priority under a PREEMPT_RT kernel (https://frankaemika.github.io/docs/installation_linux.html). After installation, the real-time kernel can be found in the grub boot menu. 

# LCM bindings
After installing LCM (https://lcm-proj.github.io/), the bindings are generated using the following command:
```
chmod 755 start_lcm.sh
ls -l start_lcm.sh
./start_lcm.sh
```
This bash file creates the bindings used by the `controller.py` to send commands and receive states from the other components of the testbed. Note that this bash file has to be run before building and running the executables, since the automatically generated functions are imported and used in the C++ and python files.

# Build Executable Files
Use following commands to build the executable files `torque_control` and `gripper_control` in the build directory:
```
mkdir build
cd build
cmake ..
make
```
Additionally, the executables `echo_robot_state`, `zero_torque`, `time_measure`, `cartesian_pose` are built.

# Motor Setup
To operate the motor, the bash file `motor_init.sh` has to be run. This initiates the communication via can port with the motor. This command has to be executed until the output says "state UP".

# Running the driver
Two different drivers were made, using different method. The first one is using the time matching method and is run using `tm_driver.sh`, while the second one is using the closed loop method and is run using `cl_driver.sh`. Both of the drivers run the `torque_control.cpp` and `gripper_control.cpp`, which remain unchanged regardless of the method used. The difference between the drivers is in the files `controller.py` and `motor_controller.py`.


```
chmod 755 tm_driver.sh
ls -l tm_driver.sh
./tm_driver.sh
chmod 755 dcl_driver.sh
ls -l cl_driver.sh
./cl_driver.sh
```
