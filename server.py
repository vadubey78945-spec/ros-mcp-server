from mcp.server.fastmcp import FastMCP
import socket
import websocket
import json
from typing import List, Tuple, Dict, Any

LOCAL_IP = "192.168.50.236" # Local IP
ROSBRIDGE_IP = "192.168.50.90" # ROS Bridge Server IP
ROSBRIDGE_PORT = 9090

mcp = FastMCP("ros_mcp_server")

# Global WebSocket connection
ws_connection = None

def create_websocket_connection():
    global ws_connection
    try:
        if ws_connection is None or not ws_connection.connected:
            sock = socket.create_connection((ROSBRIDGE_IP, ROSBRIDGE_PORT), source_address=(LOCAL_IP, 0))
            ws = websocket.WebSocket()
            ws.sock = sock
            ws.connect(f"ws://{ROSBRIDGE_IP}:{ROSBRIDGE_PORT}")
            print("WebSocket connected")
            ws_connection = ws
        return ws_connection
    except Exception as e:
        print(f"Failed to connect: {e}")
        ws_connection = None
        return None

def close_websocket_connection():
    global ws_connection
    if ws_connection is not None and ws_connection.connected:
        try:
            ws_connection.close()
            print("WebSocket connection closed")
        except Exception as e:
            print(f"Error closing WebSocket connection: {e}")
        finally:
            ws_connection = None

# @mcp.tool()
# def pub_text(text: str):
#     if not text:
#         print("Error: Text cannot be empty")
#         return

#     ws = create_websocket_connection()
#     if not ws:
#         return

#     publish_message = {
#         "op": "publish",
#         "topic": "/text",
#         "msg": {"data": text}
#     }
#     try:
#         ws.send(json.dumps(publish_message))
#         print(f"Sent text: {text}")
#         close_websocket_connection()
#     except Exception as e:
#         print(f"Failed to send text message: {e}")
#         close_websocket_connection()  # 연결이 끊어진 경우 재연결을 위해 연결을 닫음


@mcp.tool()
def pub_twist(linear: float, angular: float):
    if not linear or not angular:
        print("Error: Linear and angular cannot be empty")
        return
    
    ws = create_websocket_connection()
    if not ws:
        return

    publish_message = {
        "op": "publish",
        "topic": "/cmd_vel",
        "msg": {
            "linear": {
                "x": linear,
                "y": 0,
                "z": 0
            },
            "angular": {
                "x": 0,
                "y": 0,
                "z": angular
            }
        }
    }

    try:
        ws.send(json.dumps(publish_message))
        print(f"Sent twist: linear={linear}, angular={angular}")
        close_websocket_connection()
    except Exception as e:
        print(f"Failed to send twist message: {e}")
        close_websocket_connection()

@mcp.tool()
def pub_twist_seq(linear: List[float], angular: List[float], duration: float):
    if not linear or not angular:
        print("Error: Linear and angular cannot be empty")
        return
    
    for i in range(len(linear)):
        pub_twist(linear[i], angular[i])
        time.sleep(duration)

    try:
        close_websocket_connection()
    except Exception as e:
        print(f"Error closing WebSocket connection: {e}")
        
        
