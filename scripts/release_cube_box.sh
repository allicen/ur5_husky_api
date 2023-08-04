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

# look at box
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.594390630722046, -1.8020504156695765, 1.942338466644287,
-2.2520082632647913, -1.5818265120135706, 0.00032357408781535923]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once

waitForInput

# move closer to box
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.5220543146133423, -1.0000537077533167, 1.5697979927062988,
-2.2535069624530237, -1.5818026701556605, 0.00032357408781535923]], delay:[0,0,0,0,0,20], gripperAngle: 0.0}" --once
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
