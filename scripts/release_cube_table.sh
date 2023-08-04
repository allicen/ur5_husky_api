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

# look at table
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.5307971239089966, -1.9391124884234827, 1.29396390914917,
-1.7322405020343226, -1.516792122517721, -0.0008266607867639664]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once

waitForInput

# move to table surface
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.522724986076355, -1.2508557478534144, 1.2190613746643066,
-2.815589729939596, -1.5163243452655237, -0.0006468931781213882]], delay:[0,0,0,0,0,20], gripperAngle: 0.0}" --once
sleep 3
waitForInput

# release cube
rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.085" --once
sleep 1
waitForInput

# return to Transport pos
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.595672, -2.871760, 2.799204, -3.072348,-1.581982, 0.000120]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once
sleep 3

echo "Done!"
