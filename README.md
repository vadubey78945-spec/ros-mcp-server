## Overview
![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
[![smithery badge](https://smithery.ai/badge/@lpigeon/ros_mcp_server)](https://smithery.ai/server/@lpigeon/ros_mcp_server)
![Static Badge](https://img.shields.io/badge/license-MIT-blue)

<center><img src="https://github.com/lpigeon/ros_mcp_server/blob/main/img/framework.png"/></center>
The ROS MCP Server is designed to facilitate the control of robotic movement by providing a set of functions that allow for precise manipulation of linear and angular velocities. This enables the robot to perform complex maneuvers and navigate through various environments efficiently.

## Installation

### Installing via Smithery

To install ros_mcp_server for Claude Desktop automatically via [Smithery](https://smithery.ai/server/@lpigeon/ros_mcp_server):

```bash
npx -y @smithery/cli install @lpigeon/ros_mcp_server --client claude
```

### Installing Locally

### `uv` Installation
- To install `uv`, you can use the following command:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```
or
```bash
pip install uv
```

- Create virtual environment and activate it (Optional)
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
<center><img src="https://github.com/lpigeon/ros_mcp_server/blob/main/img/how_to_use_1.png" width="500"/></center>

### 5. Check `rosbridge_server` and `ros topic`.
- `rosbridge_server`
<center><img src="https://github.com/lpigeon/ros_mcp_server/blob/main/img/how_to_use_2.png" /></center>

- `ros topic`
<center><img src="https://github.com/lpigeon/ros_mcp_server/blob/main/img/how_to_use_3.png" /></center>

## Simulation Test
MCP-based control using the MOCA mobile manipulator within the NVIDIA Isaac Sim simulation environment. 

<center><img src="https://github.com/lpigeon/ros_mcp_server/blob/main/img/result.gif" /></center>
