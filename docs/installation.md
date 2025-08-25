# Installation Guide
Installation includes the following steps:
- Install the MCP server
  - Clone this repository
  - Instal uv (Python virtual environment manager)
- Install and configure the Language model client
  - Instal any language model client (We demo with Claude Desktop)
  - Configure the client to run the MCP server and connect automatically on launch.
- Install and launch Rosbridge


Below are detailed instructions for each of these steps. 

---
# 1. Install the MCP server (On the host machine where the LLM will be running)

## 1.1. Clone the Repository

```bash
git clone https://github.com/robotmcp/ros-mcp-server.git
```

Note the **absolute path** to the cloned directory — you’ll need this later when configuring your language model client.

---

## 1.2. Install UV (Python Virtual Environment Manager)

You can install [`uv`](https://github.com/astral-sh/uv) using one of the following methods:

### Option A: Shell installer
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

### Option B: Using pip
```bash
pip install uv
```

---

# 2. Install and configure a Language Model Client 

Any LLM client that supports MCP can be used. We use **Claude Desktop** for testing and development. This section has instructions for:
- Linux (Ubuntu)
- Windows (using WSL)
- MacOS


## 2a. Linux (Ubuntu)
### Download Claude 
- From the community-supported [claude-desktop-debian](https://github.com/aaddrick/claude-desktop-debian)

### Configure the client to launch the MCP server
- Locate and edit the `claude_desktop_config.json` file:
- (If the file does not exist, create it)
```bash
code ~/.config/Claude/claude_desktop_config.json
```

- Add the following to the `"mcpServers"` section of the JSON file
- Make sure to replace `<ABSOLUTE_PATH>` with the path to your `ros-mcp-server` folder:

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": [
        "--directory",
        "/<ABSOLUTE_PATH>/ros-mcp-server",
        "run",
        "server.py"
      ]
    }
  }
}
```

---
## 2b. MacOS
### Download Claude 
- From [claude.ai](https://claude.ai/download)

### Configure the Host to launch the MCP server
- Locate and edit the `claude_desktop_config.json` file:
- (If the file does not exist, create it)
```bash
code ~/Library/Application\ Support/Claude/claude_desktop_config.json
```

- Add the following to the `"mcpServers"` section of the JSON file
- Make sure to replace `<ABSOLUTE_PATH>` with the path to your `ros-mcp-server` folder:

```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": [
        "--directory",
        "/<ABSOLUTE_PATH>/ros-mcp-server",
        "run",
        "server.py"
      ]
    }
  }
}
```

## 2c. Windows (Using WSL)
This will have Claude running on Windows and the MCP server running on WSL.

### Install WSL (Windows Subsystem for Linux)
- [Downlowd WSL](https://apps.microsoft.com/detail/9pn20msr04dw?hl=en-US&gl=US) 

### Download your client - Claude 
- From [claude.ai](https://claude.ai/download)

### Configure the client to launch the MCP server
- Locate and edit the `claude_desktop_config.json` file:
- (If the file does not exist, create it)
```bash
code ~/.config/Claude/claude_desktop_config.json
```

- Add the following to the `"mcpServers"` section of the JSON file
- Make sure to replace `<ABSOLUTE_PATH>` with the path to your `ros-mcp-server` folder:
- Set the **full WSL path** to your `uv` installation (e.g., `/home/youruser/.local/bin/uv`)
- Use the correct **WSL distribution name** (e.g., `"Ubuntu-22.04"`)
```json
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "wsl",
      "args": [
        "-d", "Ubuntu-22.04",
        "/home/youruser/.local/bin/uv",
        "--directory",
        "/<ABSOLUTE_PATH>/ros-mcp-server",
        "run",
        "server.py"
      ]
    }
  }
}
```

# 3. Test that the Client (Claude) can connect to the MCP server

- Launch your host and check connection status. 
- Below is an image of what that looks like in Claude. The ros-mcp-server should be visible in your list of tools.

<p align="center">
  <img src="https://github.com/robotmcp/ros-mcp-server/blob/main/img/connected_mcp.png" width="500"/>
</p>
---

# 4. Install and run rosbridge (On the target robot Where ROS will be running)

## 3.1. Install `rosbridge_server`

This package is required for MCP to interface with ROS or ROS 2 via WebSocket. It needs to be installed on the same machine that is running ROS.

```bash
sudo apt install ros-${ROS_DISTRO}-rosbridge-server
```
Examples:
```bash
sudo apt install ros-noetic-rosbridge-server
```
```bash
sudo apt install ros-humble-rosbridge-server
```

## 3.2. Launch rosbridge in your ROS environment:
```bash
# ROS 1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS 2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

> ⚠️ Don’t forget to `source` your ROS workspace before launching, especially if you're using custom messages or services.

---
