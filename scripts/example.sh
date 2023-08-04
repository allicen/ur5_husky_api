#!/bin/bash

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

echo "Joints positions:"
echo $(get_joints_positions)


