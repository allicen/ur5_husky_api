#!/bin/bash

function waitForInput() {
  echo $1
  while true; do
    read -p "Press y to continue and n for end script " yn
    case $yn in
      [Yy]* ) echo "Ok"; break;;
      [Nn]* ) exit;;
      * ) echo "Please only Y/y or N/n"
    esac
  done
}

function get_transformed_string() {
  local ros_message="$1"
  local transformed_string=""

  # Extract the numbers between the '[' and ']' characters
  if [[ $ros_message =~ \[([^]]+)\] ]]; then
    # Get the matched substring
    local numbers="${BASH_REMATCH[1]}"

    # Remove any whitespace characters
    numbers=$(echo "$numbers" | tr -d '[:space:]')

    # Replace the ',' separator with ', '
    transformed_string=${numbers//,/,\ }

    # Append a newline character at the end
    # transformed_string+="\n"
  fi

  echo "$transformed_string"
}

function get_joints_positions() {
  ros_message=$(rostopic echo -n 1 /set_joints_positions)
  transformed_message=$(get_transformed_string "$ros_message")
  echo $transformed_message
}

goal_joints_positions=$(get_joints_positions)

# return to Transport pos
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.595672, -2.871760, 2.799204, -3.072348,-1.581982, 0.000120]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once


# Start scenario
echo "--------------------"
waitForInput "START RECORDING ROSBAG NOW"
waitForInput "DoubleCheck - Is rosbag recording?"

echo "Scenario Started..."

# look at cube
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.4775620698928833, -1.6089142004596155, 1.1463332176208496,
-1.400895897542135, -1.5800517241107386, 0.00013182648399379104]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once

waitForInput

# move to cube
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[$goal_joints_positions]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once

# rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[1.47336, -0.490893, 1.58026, -2.72144, -1.5576, 0.000203732]], delay:[0,0,0,0,0,20], gripperAngle: 0.0}" --once
sleep 3
waitForInput

# get cube
rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.04" --once
waitForInput

# return to Transport pos
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.595672, -2.871760, 2.799204, -3.072348,-1.581982, 0.000120]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once
sleep 3

echo "Done!"
