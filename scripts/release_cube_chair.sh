#!/bin/bash

function waitForInput {
  while true; do
    read -p "Press y to continue and n for end script " yn
    case $yn in
      [Yy]* ) echo "Ok"; break;;
      [Nn]* ) exit;;
      * ) echo "Please only Y/y or N/n"
    esac
  done
}

# look at chair
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.4105781316757202, -2.2127626577960413, 1.8677821159362793,
-2.1619752089129847, -1.41978627840151, -0.0006588141070764664]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once

waitForInput

# move to chair surface
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.4123626947402954, -1.396700684224264, 1.7271356582641602,
-2.2998464743243616, -1.4468653837787073, -0.0006468931781213882]], delay:[0,0,0,0,0,20], gripperAngle: 0.0}" --once
sleep 3
waitForInput

# release cube
rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.085" --once
waitForInput

# return to Transport pos
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.595672, -2.871760, 2.799204, -3.072348,-1.581982, 0.000120]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once
sleep 3

echo "Done!"
