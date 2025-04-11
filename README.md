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
  - `linear`: Linear velocity (float)
  - `angular`: Angular velocity (float)

### pub_twist_seq
- **Purpose**: Sends a sequence of movement commands to the robot, allowing for multi-step motion control.
- **Parameters**:
  - `linear`: List of linear velocities (number[])
  - `angular`: List of angular velocities (number[])
  - `duration`: List of durations for each step (number[])