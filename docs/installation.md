# Installation

## Clone repository
Clone this repository. Note the abslute path to me used later in the configuration file. 

## Install `uv` 
- To install `uv`, you can use the following command:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```
or
```bash
pip install uv
```

## Install Language Model Client
Any language model client that supports MCP can be used. We use Claude Desktop as an example. 

- Windows/MacOS installation from [claude.ai](https://claude.ai/download).
- Linux workaround from [claude-desktop-debian](https://github.com/aaddrick/claude-desktop-debian).


## MCP Server Configuration
In your language model client, set MCP setting to mcp.json.
Make sure to set the path to your cloned repository. 

```bash
{
  "mcpServers": {
    "ros-mcp-server": {
      "command": "uv",
      "args": [
        "--directory",
        "/<ABSOLUTE_PATH_TO_PARENT_FOLDER>/ros-mcp-server",
        "run",
        "server.py"
      ]
    }
  }
}
```


If you use Claude Desktop, you can find mcp.json using the following command:

- MacOS
```bash
code ~/Library/Application\ Support/Claude/claude_desktop_config.json
```

- Linux(Ubuntu)
```bash
code ~/.config/Claude/claude_desktop_config.json
```

- Windows
```bash
code $env:AppData\Claude\claude_desktop_config.json
```
Note: This does work with Claude on windows with the server running on WSL.