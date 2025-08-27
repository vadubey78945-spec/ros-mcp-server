# ROS MCP Server 🧠⇄🤖

![Static Badge](https://img.shields.io/badge/ROS-Available-green)
![Static Badge](https://img.shields.io/badge/ROS2-Available-green)
![Static Badge](https://img.shields.io/badge/License-Apache%202.0-blue)

<p align="center">
  <img src="https://github.com/robotmcp/ros-mcp-server/blob/main/docs/images/framework.png"/>
</p>

The **ROS MCP Server** bridges large language models (LLMs) with robot control, allowing users to issue natural language commands that are translated into ROS/ROS2 instructions. It is compatible with any language model that supports MCP, can run with both **ROS** and **ROS2**, and communicatess via WebSocket for cross-platform compatibility. 

It does not require changes to existing robot code, since it is built on `rosbridge` — making it fast and easy to integrate into any robotic stack.

---
## Highlights

- **Universal compatibility**: Works with both ROS and ROS2 via rosbridge
- **Cross-platform support**: Compatible with Linux, Windows, and macOS
- **AI-ready**: Seamlessly integrates with LLMs and AI systems to drive robot behavior via natural language
- **No ROS node modification required**: Fully interoperable with existing systems

---
## Features

- List topics, services, and types
- View message type details (incl. custom)
- Publish/subscribe to any topic (incl. custom)
- Call any service (incl. custom)
- Get/set parameters
- (Coming soon): Full Action support
- (Coming soon): Permission controls for write access

---
# Examples in action:
**Controlling the MOCA mobile manipulator in the NVIDIA Isaac Sim environment.** In this example, the user inputs commands directly into Claude desktop, which can now use the MCP server to directly control a robot simulated in Isaac Sim.

<p align="center">
  <img src="https://github.com/robotmcp/ros-mcp-server/blob/main/docs/images/result.gif" />
</p>


---

# Getting Started
Below are instructions to install and run the MCP server with your robot. The MCP server is agnostic to the version of ROS/ROS2 that you are running and works with any LLM that supports MCP. 

Our examples use Claude desktop as the LLM client, but any client that supports the MCP protocol can be used!

<p align="center">
  <img src="https://github.com/robotmcp/ros-mcp-server/blob/main/docs/images/MCP_topology.png"/>
</p>

## Installation and Use

Follow the [installation guide](docs/installation.md) for full setup instructions on:
- Cloning the ROS-MCP-server repository
- Installing `uv` and `rosbridge`
- Installing `Claude desktop`
- Configuring Claude to connect to the ROS-MCP-server.
- Installing and starting rosbridge on the target robot

---


## More Examples and Tutorials
Look into our [list of examples](docs/examples.md) for inspiration and tutorials of the ROS-MCP server in use. Feel free to submit PRs adding examples that you have successfully implemented!

## Contributing

Contributions are welcome!  
Whether you're fixing a typo, adding a feature, or suggesting an improvement — thank you.  

Please see the [contributing guidelines](docs/contributing.md) to get started.

---

## License

This project is licensed under the [Apache License 2.0](LICENSE).
