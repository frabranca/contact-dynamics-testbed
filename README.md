## Contact Dynamics Test Bed

# Prerequisites for Franka Robot Arm Driver
The Franka PANDA research robot is controlled with LCM bindings between C++ and Python. The C++ codes use the franka library (https://frankaemika.github.io/docs/libfranka.html).

To operate the robot libfranka has to be installed, while for real time communication with the robot the controller program on the workstation PC must run with real-time priority under a PREEMPT_RT kernel (https://frankaemika.github.io/docs/installation_linux.html).

# Run Executable Files
Use following commands to build the executable file `#torque_control` in the build directory:
```
mkdir build
cd build
cmake ..
make
```
After installing LCM (https://lcm-proj.github.io/), the bindings are generated using the following commands:

```
lcm-gen -xp command.lcm
lcm-gen -xp state.lcm
```

Initiate the LCM communication between the files `#controller_class.py` and `#torque_control.cpp` by running the following commands in this specific order:

```
python3 controller_class.py
sudo ./torque_control <fci-address>
```

