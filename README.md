## Overview

The ROS MCP Server is designed to facilitate the control of robotic movement by providing a set of functions that allow for precise manipulation of linear and angular velocities. This enables the robot to perform complex maneuvers and navigate through various environments efficiently.

## MCP Functions

### pub_twist
- **Purpose**: Sends movement commands to the robot by setting linear and angular velocities.
- **Parameters**:
  - `linear`: Linear velocity (float)
  - `angular`: Angular velocity (float)

### pub_twist_seq
- **Purpose**: Sends a sequence of movement commands to the robot, allowing for multi-step motion control.
- **Parameters**:
  - `linear`: List of linear velocities (number[])
  - `angular`: List of angular velocities (number[])
  - `duration`: List of durations for each step (number[])