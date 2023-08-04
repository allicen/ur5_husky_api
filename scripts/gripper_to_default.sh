#!/bin/bash

# gripper to default
rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.085" --once


echo "Done!"
