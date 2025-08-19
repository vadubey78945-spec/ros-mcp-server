# Installation Guide

Follow these steps to set up the ROS MCP Server and connect it with a supported language model client (e.g., Claude Desktop).

---
# On the Host Machine (Where the LLM will be running)

## 1. Clone the Repository

```bash
git clone https://github.com/robotmcp/ros-mcp-server.git
```

Note the **absolute path** to the cloned directory — you’ll need this later when configuring your language model client.

---

## 2. Install UV (Python Virtual Environment Manager)

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

## 3. Install a Language Model Client

Any LLM client that supports MCP can be used. We use **Claude Desktop** for testing and development.

- **MacOS / Windows**: Download from [claude.ai](https://claude.ai/download)
- **Linux (Unofficial)**: Use the community-supported [claude-desktop-debian](https://github.com/aaddrick/claude-desktop-debian)

---

## 4. Configure the Language Model Client (Specific to Claude Desktop)

### Locate and edit the `claude_desktop_config.json` file:

- **MacOS**
```bash
code ~/Library/Application\ Support/Claude/claude_desktop_config.json
```

- **Linux (Ubuntu)**
```bash
code ~/.config/Claude/claude_desktop_config.json
```

- **Windows (PowerShell)**
```powershell
code $env:AppData\Claude\claude_desktop_config.json
```

### Add the following to the `"mcpServers"` section of the JSON file
Make sure to replace `<ABSOLUTE_PATH>` with the path to your `ros-mcp-server` folder:

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

✅ **Tip:** If on Windows, using [WSL](https://apps.microsoft.com/detail/9pn20msr04dw?hl=en-US&gl=US) is also a strong option - with Claude running on Windows and the MCP server running on WSL. 

Use the below configuration for this, making sure to: 
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
---
## 5. Test that the Host (Claude) can connect to the MCP server

- Launch your host and check connection status. 
- Below is an image of what that looks like in Claude. The ros-mcp-server should be visible in your list of tools.

<p align="center">
  <img src="https://github.com/robotmcp/ros-mcp-server/blob/main/img/connected_mcp.png"/>
</p>
---

# On the Target Robot (Where ROS will be running)

## 1. Install `rosbridge_server`

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

## Test rosbridge server by launching in your ROS environment:
```bash
# ROS 1
roslaunch rosbridge_server rosbridge_websocket.launch

# ROS 2
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

> ⚠️ Don’t forget to `source` your ROS workspace before launching, especially if you're using custom messages or services.

---
