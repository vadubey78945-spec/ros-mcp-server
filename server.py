from mcp.server.fastmcp import FastMCP
import socket
import websocket
import json
from typing import List, Any
import time

LOCAL_IP = "192.168.50.236"
ROSBRIDGE_IP = "192.168.50.90"
ROSBRIDGE_PORT = 9090

mcp = FastMCP("ros-mcp-server")
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

def to_float(value: Any) -> float:
    try:
        return float(value)
    except (ValueError, TypeError):
        raise ValueError(f"Invalid value for float conversion: {value}")

@mcp.tool()
def pub_twist(linear: List[Any], angular: List[Any]):
    try:
        linear_floats = [to_float(l) for l in linear]
        angular_floats = [to_float(a) for a in angular]
    except ValueError as e:
        print(e)
        return

    ws = create_websocket_connection()
    if not ws:
        return

    publish_message = {
        "op": "publish",
        "topic": "/cmd_vel",
        "msg": {
            "linear": {"x": linear_floats[0], "y": 0, "z": 0},
            "angular": {"x": 0, "y": 0, "z": angular_floats[2]}
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
def pub_twist_seq(linear: List[Any], angular: List[Any], duration: List[Any]):
    try:
        linear_floats = [to_float(l) for l in linear]
        angular_floats = [to_float(a) for a in angular]
        duration_floats = [to_float(d) for d in duration]
    except ValueError as e:
        print(e)
        return

    for l, a, d in zip(linear_floats, angular_floats, duration_floats):
        l = [l, 0, 0]
        a = [0, 0, a]
        pub_twist(l, a)
        time.sleep(d)

    try:
        close_websocket_connection()
    except Exception as e:
        print(f"Error closing WebSocket connection: {e}")

if __name__ == "__main__":
    # Initialize and run the server
    mcp.run(transport='stdio')
