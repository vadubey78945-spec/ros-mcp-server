from mcp.server.fastmcp import FastMCP
from typing import List, Any, Optional
from pathlib import Path
import json
from utils.websocket_manager import WebSocketManager

# ROS message wrapper imports
from msgs.geometry_msgs import Twist
from msgs.sensor_msgs import Image, JointState

# ROS bridge connection settings
LOCAL_IP = "127.0.0.1"      # Replace with your local IP address. Default is localhost.
ROSBRIDGE_IP = "127.0.0.1"  # Replace with your rosbridge server IP address. Default is localhost.
ROSBRIDGE_PORT = 9090

# Initialize MCP server and WebSocket manager
mcp = FastMCP("ros-mcp-server")
ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)

# Mapping of supported message types to their wrapper classes
MESSAGE_TYPE_MAP = {
    "Twist": Twist,
    "Image": Image,
    "JointState": JointState
}

@mcp.tool()
def get_topics() -> dict:
    """
    Fetch available topics from the ROS bridge.

    Returns:
        dict: Contains two lists - 'topics' and 'types',
            or a message string if no topics are found.
    """
    message = {
        "op": "call_service",
        "service": "/rosapi/topics",
        "id": "get_topics_request_1"
    }
    
    response = ws_manager.request(message)
    ws_manager.close()

    if response and "values" in response:
        return response["values"]
    else:
        return {"warning": "No topics found"}

@mcp.tool()
def get_supported_message_types()-> List[str]:
    """
    List all message types supported by this MCP server.

    Returns:
        list: Names of supported message types, e.g. ['Twist', 'Image', 'JointState'].
    """
    return list(MESSAGE_TYPE_MAP.keys())

@mcp.tool()
def publish_message(msg_type: str, topic: str, data: dict)-> str:
    """
    Dynamically publish a ROS message.

    msg_type: one of types returned by 'get_supported_message_types()'
    topic: ROS topic name, e.g. '/cmd_vel' or '/robot_pose'
    data: dictionary with message-specific fields
          - Twist requires: { "linear": [x, y, z], "angular": [x, y, z] }
          - JointState requires: { "name": [...], "position": [...], "velocity": [...], "effort": [...] }

    """
    if msg_type not in MESSAGE_TYPE_MAP:
        return f"Unsupported message type: {msg_type}"

    msg_class = MESSAGE_TYPE_MAP[msg_type]
    msg_instance = msg_class(ws_manager, topic=topic)

    # Twist message handling
    if msg_type == "Twist":
        linear = data.get("linear", [0.0, 0.0, 0.0])
        angular = data.get("angular", [0.0, 0.0, 0.0])
        msg = msg_instance.publish(linear, angular)

    # JointState message handling
    elif msg_type == "JointState":
        name = data.get("name", [])
        position = data.get("position", [])
        velocity = data.get("velocity", [])
        effort = data.get("effort", [])
        msg = msg_instance.publish(name, position, velocity, effort)

    else:
        # Message types like Image are usually only subscribed
        msg = None

    ws_manager.close()
    return f"{msg_type} message published to {topic}" if msg else f"Failed to publish {msg_type}"

@mcp.tool()
def pub_twist_seq(topic: str, data: dict)-> str:
    """
    Publish a sequence of Twist messages to a specified topic.

    Args:
        topic (str): The ROS topic name to publish to, e.g. '/cmd_vel'.
        data (dict): A dictionary containing:
            - "linear_seq": List of [x, y, z] linear velocity vectors.
            - "angular_seq": List of [x, y, z] angular velocity vectors.
            - "duration_seq": List of durations in seconds for each step.

            Example:
            {
                "linear_seq": [[0.1, 0.0, 0.0], [0.2, 0.0, 0.0]],
                "angular_seq": [[0.0, 0.0, 0.1], [0.0, 0.0, 0.2]],
                "duration_seq": [2.0, 3.0]
            }

    Returns:
        str: Confirmation message after publishing the sequence.
    """
    from msgs.geometry_msgs import Twist as TwistMsg

    # Extract fields with defaults
    linear_seq = data.get("linear_seq", [])
    angular_seq = data.get("angular_seq", [])
    duration_seq = data.get("duration_seq", [])

    # Create a Twist instance for the given topic
    twist_instance = TwistMsg(ws_manager, topic=topic)
    twist_instance.publish_sequence(linear_seq, angular_seq, duration_seq)

    # Close WebSocket after publishing
    ws_manager.close()

    return f"Published Twist sequence to topic {topic}"


@mcp.tool()
def subscribe_once(topic_name: str, topic_type: str, timeout: float = 2.0) -> dict:
    """
    Subscribe to a given ROS topic via rosbridge and return the first message received.

    Args:
        topic_name (str): The ROS topic name (e.g., "/cmd_vel", "/joint_states").
        topic_type (str): The ROS message type (e.g., "geometry_msgs/Twist").
        timeout (float): How long (in seconds) to wait for a message before returning an error.

    Returns:
        dict:
            - {"msg": <parsed ROS message>} if successful
            - {"error": "<error message>"} if subscription or timeout fails
    """
    # Construct the rosbridge subscribe message
    subscribe_msg = {
        "op": "subscribe",
        "topic": topic_name,
        "type": topic_type,
        "queue_length": 1  # request just one message
    }

    # Send subscription request & wait for a response
    response = ws_manager.request(subscribe_msg, timeout=timeout)

    # Always close the websocket after a single-shot subscription
    ws_manager.close()

    # Handle rosbridge response
    if "error" in response:
        return response  # structured error from request()
    
    # If we got a valid ROS message
    if "msg" in response:
        return {"msg": response["msg"]}

    # If the response is something unexpected, return raw
    return {"error": "unexpected_response_format", "raw": response}


if __name__ == "__main__":
    mcp.run(transport="stdio")
