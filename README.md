## Contact Dynamics Test Bed

# Franka Robot Arm Driver
The Franka PANDA research robot is controlled with LCM bindings between C++ and Python. The C++ codes use the franka library (https://frankaemika.github.io/docs/libfranka.html).

To operate the robot libfranka has to be installed, while for real time communication with the robot the controller program on the workstation PC must run with real-time priority under a PREEMPT_RT kernel (https://frankaemika.github.io/docs/installation_linux.html). After installation, the real-time kernel can be found in the grub boot menu. 

# Run Executable Files
Use following commands to build the executable files `torque_control` and `gripper_control` in the build directory:
```
mkdir build
cd build
cmake ..
make
```
After installing LCM (https://lcm-proj.github.io/), the bindings are generated using the following command:
```
chmod 755 build-lcm.sh
ls -l build-lcm.sh
./build-lcm.sh
```

Initiate the LCM communication between the files `controller.py`, `torque_control.cpp` and `gripper_control.cpp` by running the following command:

```
chmod 755 driver.sh
ls -l driver.sh
./driver.sh
```

