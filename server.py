from mcp.server.fastmcp import FastMCP
from typing import List, Any, Optional
from pathlib import Path
from utils.websocket_manager import WebSocketManager
from msgs.geometry_msgs import Twist
from msgs.sensor_msgs import Image

LOCAL_IP = "192.168.50.236"
ROSBRIDGE_IP = "192.168.50.90"
ROSBRIDGE_PORT = 9090

mcp = FastMCP("ros-mcp-server")
ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)
twist = Twist(ws_manager, topic="/cmd_vel")
image = Image(ws_manager, topic="/camera/image_raw")


@mcp.tool()
def pub_twist(linear: List[Any], angular: List[Any]):
    msg = twist.publish(linear, angular)
    ws_manager.close()
    
    if msg is not None:
        return "Twist message published successfully"
    else:
        return "No message published"

@mcp.tool()
def pub_twist_seq(linear: List[Any], angular: List[Any], duration: List[Any]):
    twist.publish_sequence(linear, angular, duration)


@mcp.tool()
def sub_image():
    msg = image.subscribe()
    ws_manager.close()
    
    if msg is not None:
        return "Image data received and downloaded successfully"
    else:
        return "No image data received"

if __name__ == "__main__":
    mcp.run(transport="stdio")
