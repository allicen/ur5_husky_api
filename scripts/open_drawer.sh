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

# return to safe position
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.5383899211883545, -1.2902405897723597, 2.4504973888397217, -4.262852017079489, -1.5552275816546839, -1.5088165442096155]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once
sleep 3
waitForInput


# return to Transport pos
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.595672, -2.871760, 2.799204, -3.072348,-1.581982, 0.000120]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once


# Start scenario
echo "--------------------"
waitForInput "START RECORDING ROSBAG NOW"
waitForInput "DoubleCheck - Is rosbag recording?"

echo "Scenario Started..."

# look at drawer
# rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
# [position:[1.595684289932251, -2.8716395537005823, 2.79904842376709, -3.073042694722311, -1.5786007086383265, -1.5424135367022913]], delay: [0,0,0,0,0,20],
# gripperAngle: 0.0}" --once

# waitForInput

# move to safe position
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.5383899211883545, -1.2902405897723597, 2.4504973888397217, -4.262852017079489, -1.5552275816546839, -1.5088165442096155]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once
sleep 3
waitForInput

# move to drawer
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions: [position:[$goal_joints_positions]], delay:[0,0,0,0,0,20], gripperAngle: 0.0}" --once
sleep 3
waitForInput "Grab the drawer"

# grab drawer
rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.008" --once
waitForInput "Move the platform back to open the drawer"

# release drawer
waitForInput "Release the drawer"
rostopic pub /gripper_angle gripper_move/GripperAngle "angle: 0.085" --once
waitForInput

# move to safe position
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.5383899211883545, -1.2902405897723597, 2.4504973888397217, -4.262852017079489, -1.5552275816546839, -1.5088165442096155]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once
sleep 3

waitForInput

# return to Transport pos
rostopic pub move_robot_delay_gripper ur5_info/MoveUR5WithGripper "{positions:
[position:[1.595672, -2.871760, 2.799204, -3.072348,-1.581982, 0.000120]], delay: [0,0,0,0,0,20],
gripperAngle: 0.0}" --once
sleep 3
echo "Done!"
