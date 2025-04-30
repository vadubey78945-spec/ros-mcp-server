from mcp.server.fastmcp import FastMCP
from msgs.geometry_msgs import TwistMessage
from utils.websocket_manager import WebSocketManager

LOCAL_IP = "192.168.50.236"
ROSBRIDGE_IP = "192.168.50.90"
ROSBRIDGE_PORT = 9090

mcp = FastMCP("ros-mcp-server")
ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
twist = TwistMessage(ws_manager)


@mcp.tool()
def pub_twist(linear, angular):
    twist.publish(linear, angular)
    ws_manager.close()


@mcp.tool()
def pub_twist_seq(linear, angular, duration):
    twist.publish_sequence(linear, angular, duration)


if __name__ == "__main__":
    mcp.run(transport="stdio")
