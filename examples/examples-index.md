# ROS MCP Server Examples

This directory contains examples demonstrating how to use the ROS MCP Server with different robotic systems and simulators.

## Available Examples

### 🐢 [Turtlesim Example](turtlesim/)

**Difficulty**: Beginner  
**Description**: A simple example using ROS2 Turtlesim - the classic "Hello World" of ROS robotics.

**What you'll learn**:
- How to set up a Docker container with ROS2 and Turtlesim
- How to automatically launch Turtlesim with ROS Bridge WebSocket server
- How to connect the MCP server to control a simulated robot
- Basic ROS2 topic and service interaction

**Prerequisites**: Docker and Docker Compose

**Quick Start**:
```bash
cd examples/turtlesim
docker compose up -d
```

**Features**:
- ✅ Automatic Docker setup with ROS2 Humble
- ✅ Turtlesim with visual interface
- ✅ ROS Bridge WebSocket server (ws://localhost:9090)
- ✅ Turtle teleop for manual control
- ✅ MCP server integration ready

---

## Getting Started with Examples

Each example is self-contained and includes:
- **Docker setup** (where applicable) for easy deployment
- **Step-by-step instructions** for setup and testing
- **Troubleshooting guides** for common issues
- **Integration instructions** for the MCP server

## Contributing Examples

We welcome contributions! If you have a working example with the ROS MCP Server, please:

1. Create a new subdirectory in `examples/`
2. Include a comprehensive README or tutorial
3. Add Docker setup if applicable
4. Test thoroughly before submitting

See our [contributing guidelines](../docs/contributing.md) for more details.

## Example Categories

Examples are organized by complexity and use case:

- **Beginner**: Simple simulators and basic robot control
- **Intermediate**: Real robots and complex simulations  
- **Advanced**: Multi-robot systems and custom integrations

---

## Need Help?

- Check the [main documentation](../docs/)
- Review the [installation guide](../docs/installation.md)
- Open an issue on GitHub for bugs or feature requests
- Join our community discussions

---

*Last updated: August 2024*
