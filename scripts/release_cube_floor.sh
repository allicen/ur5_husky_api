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



# move closer to box
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[0.14330361783504486, -0.6034005323993128, 1.9890742301940918, -2.935913864766256, -1.6333087126361292, 0.20578600466251373]], delay:[0,0,0,0,0,20], gripperAngle: 0.0}" --once
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
