# ROS MCP Server

![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
![Static Badge](https://img.shields.io/badge/License-Apache%202.0-blue)

<p align="center">
  <img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/framework.png"/>
</p>

The **ROS MCP Server** bridges large language models (LLMs) with robot control, allowing users to issue natural language commands that are translated into ROS/ROS2 instructions. Built on `rosbridge`, it supports both **ROS** and **ROS2**, and communicates via WebSocket for cross-platform compatibility. No changes to existing robot code are required — making it fast and easy to integrate into any robotic stack.

---

## Supported ROS Commands

- **View all running ROS topics and types**
- **View message type details**, including custom messages — no code changes needed
- **Publish and subscribe to any topic**, including custom message types
- **View all active publishers, subscribers, and nodes**
- **View all running ROS services**
- **Call any ROS service**, including custom service types
- **Get and set ROS parameters**
- *(Coming soon)*: Full ROS Action support
- *(Coming soon)*: Permission management to restrict write access to selected topics and services

---

## Highlights

- **Universal compatibility**: Works with both ROS and ROS2 via rosbridge
- **Cross-platform support**: Compatible with Linux, Windows, and macOS
- **AI-ready**: Seamlessly integrates with LLMs and AI systems to drive robot behavior via natural language
- **No ROS node modification required**: Fully interoperable with existing systems

---

## Installation

Follow the [installation guide](docs/installation.md) for full setup instructions.

---

## How To Use

### 1. Run the rosbridge server
Execute the below command on the same machine that is running your ROS processes. Source your ROS workspace first in order to have the MCP server access custom message and service types. 
#### ROS 1:
```bash
roslaunch rosbridge_server rosbridge_websocket.launch
```

#### ROS 2:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

---

### 2. Run any AI system with the `ros-mcp-server` installed

This could be your LLM wrapper, chatbot, or any custom client. We’ve used Claude Desktop (an LLM interface) for the examples shown below, but any AI system that supports MCP will work.

---

### 3. If not running locally, configure IP

- If the MCP server and rosbridge are running on the same machine, skip this step.
- If rosbridge is on another machine, this MCP server contains a tool to dynamically set the target IP.

Example:
```plaintext
My IP is 100.xx.xx.xx. Connect the ROS MCP server to 100.xx.xx.xx port 9090.
```

You can also optionally edit `server.py` and update the following values:

- `LOCAL_IP` (default: `'localhost'`)
- `ROSBRIDGE_IP` (default: `'localhost'`)
- `ROSBRIDGE_PORT` (default: `9090`)
---

### 4. You're ready to go! 
You can test out your server with some of the examples below

## Example Uses

### - Natural language commands

Example:  
```plaintext
Make the robot move forward.
```

<p align="center">
  <img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/how_to_use_1.png" width="500"/>
</p>

---

### - Monitor your ROS system

#### rosbridge server logs:
<p align="center">
  <img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/how_to_use_2.png" />
</p>

#### ROS topic output:
<p align="center">
  <img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/how_to_use_3.png" />
</p>

---

## Simulation Examples

MCP in action: controlling the MOCA mobile manipulator in the NVIDIA Isaac Sim environment.

<p align="center">
  <img src="https://github.com/lpigeon/ros-mcp-server/blob/main/img/result.gif" />
</p>

---

## Contributing

Contributions are welcome!  
Whether you're fixing a typo, adding a feature, or suggesting an improvement — thank you.  

Please see the [contributing guidelines](docs/contributing.md) to get started.

---

## License

This project is licensed under the [Apache License 2.0](LICENSE).
