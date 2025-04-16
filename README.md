## Overview

The ROS MCP Server is designed to facilitate the control of robotic movement by providing a set of functions that allow for precise manipulation of linear and angular velocities. This enables the robot to perform complex maneuvers and navigate through various environments efficiently.

## Installation

### `uv` Installation
- To install `uv`, you can use the following command:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```
or
```bash
pip install uv
```

- Create virtual environment and activate it
```bash
uv venv
source .venv/bin/activate
```

### MCP Server Configuration
Set MCP setting to mcp.json.
```bash
"ros_mcp_server": {
  "command": "uv",
  "args": [
    "--directory",
    "/ABSOLUTE/PATH/TO/PARENT/FOLDER/ros_mcp_server",,
    "run",
    "server.py"
  ]
}
```

## MCP Functions
### pub_twist
- **Purpose**: Sends movement commands to the robot by setting linear and angular velocities.
- **Parameters**:
  - `linear`: Linear velocity (List[Any])
  - `angular`: Angular velocity (List[Any])

### pub_twist_seq
- **Purpose**: Sends a sequence of movement commands to the robot, allowing for multi-step motion control.
- **Parameters**:
  - `linear`: List of linear velocities (List[Any])
  - `angular`: List of angular velocities (List[Any])
  - `duration`: List of durations for each step (List[Any])


## How To Use
### 1. Set IP and Port to connect rosbridge.
- Open `server.py` and change your `LOCAL_IP`, `ROSBRIDGE_IP` and `ROSBRIDGE_PORT`. (`ROSBRIDGE_PORT`'s default value is `9090`)

### 2. Run rosbridge server.
ROS 1
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```
ROS 2
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 3. Run any AI system that has imported ros_mcp_server.

### 4. Type "Make the robot move forward.".

### 5. Check `rosbridge_server` and `ros topic`.

## Simulation Test
MCP-based control using the MOCA mobile manipulator within the NVIDIA Isaac Sim simulation environment. 