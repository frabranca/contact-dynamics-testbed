# Test 3
A few changes were made to make the experiment more repeatable and the controller more robust.
The robot closed control loop has 3 phases:
1) waiting phase
2) trajectory phase
3) damping phase
For each of these phases different gains are used. 

Moreover the satellite was detached and attached again to the motor, which means that the contact point changed slightly. To correct for this, the position equation was adjusted to match the new contact point location.

