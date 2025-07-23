import json
import time

from mcp.server.fastmcp import FastMCP
from utils.websocket_manager import WebSocketManager

# ROS bridge connection settings
LOCAL_IP = "127.0.0.1"  # Replace with your local IP address. Default is localhost.
ROSBRIDGE_IP = "127.0.0.1"  # Replace with your rosbridge server IP address. Default is localhost.
ROSBRIDGE_PORT = 9090

# Initialize MCP server and WebSocket manager
mcp = FastMCP("ros-mcp-server")
ws_manager = WebSocketManager(ROSBRIDGE_IP, ROSBRIDGE_PORT, LOCAL_IP)


@mcp.tool()
def get_topics() -> dict:
    """
    Fetch available topics from the ROS bridge.

    Returns:
        dict: Contains two lists - 'topics' and 'types',
            or a message string if no topics are found.
    """
    # rosbridge service call to get topic list
    message = {"op": "call_service", "service": "/rosapi/topics", "id": "get_topics_request_1"}

    # Request topic list from rosbridge
    response = ws_manager.request(message)
    ws_manager.close()

    # Return topic info if present
    if response and "values" in response:
        return response["values"]
    else:
        return {"warning": "No topics found"}


@mcp.tool()
def subscribe_once(topic_name: str = "", topic_type: str = "", timeout: float = 2.0) -> dict:
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
    # Validate critical args before attempting subscription
    if not topic_name or not topic_type:
        return {"error": "Missing required arguments: topic_name and topic_type must be provided."}

    # Construct the rosbridge subscribe message
    subscribe_msg = {
        "op": "subscribe",
        "topic": topic_name,
        "type": topic_type,
        "queue_length": 1,  # request just one message
    }

    # Send subscription request & wait for a single response
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


@mcp.tool()
def publish_once(topic: str = "", msg_type: str = "", msg: dict = None, timeout: float = 2.0) -> dict:
    """
    Publish a single message to a ROS topic via rosbridge.

    Args:
        topic (str): ROS topic name (e.g., "/cmd_vel")
        msg_type (str): ROS message type (e.g., "geometry_msgs/Twist")
        msg (dict): Message payload as a dictionary
        timeout (float): Seconds to wait for an optional response. Default = 2.0

    Returns:
        dict:
            - {"success": True} if sent without errors
            - {"error": "<error message>"} if connection/send failed
            - If rosbridge responds (usually it doesn’t for publish), parsed JSON or error info
    """
    # Validate critical args before attempting publish
    if not topic or not msg_type or msg is None:
        return {"error": "Missing required arguments: topic, msg_type, and msg must all be provided."}

    # Construct rosbridge publish message
    publish_msg = {"op": "publish", "topic": topic, "msg": msg}

    # Send the message via ws_manager
    send_error = ws_manager.send(publish_msg)
    if send_error:
        ws_manager.close()
        return {"error": send_error}

    # rosbridge typically does NOT respond to publish requests
    # But we can still attempt to receive in case of errors
    response = ws_manager.receive(timeout=timeout)

    # Always close after sending
    ws_manager.close()

    # No response is normal for publish
    if response is None or response.strip() == "":
        return {
            "success": True,
            "note": "No response is expected for publish, so we have no confirmation that the message was published.",
        }

    # If rosbridge *did* send something back, parse it
    try:
        return json.loads(response)
    except json.JSONDecodeError:
        return {"error": "invalid_json", "raw": response}


@mcp.tool()
def subscribe_for_duration(
    topic_name: str = "",
    topic_type: str = "",
    duration: float = 5.0,
    max_messages: int = 100
) -> dict:
    """
    Subscribe to a ROS topic via rosbridge for a fixed duration and collect messages.

    Args:
        topic_name (str): ROS topic name (e.g. "/cmd_vel", "/joint_states")
        topic_type (str): ROS message type (e.g. "geometry_msgs/Twist")
        duration (float): How long (seconds) to listen for messages
        max_messages (int): Maximum number of messages to collect before stopping

    Returns:
        dict:
            {
                "topic": topic_name,
                "collected_count": N,
                "messages": [msg1, msg2, ...]
            }
    """
    # Validate critical args before subscribing
    if not topic_name or not topic_type:
        return {"error": "Missing required arguments: topic_name and topic_type must be provided."}

    # Send subscription request
    subscribe_msg = {
        "op": "subscribe",
        "topic": topic_name,
        "type": topic_type,
        "queue_length": 10  # allow some buffering
    }

    send_error = ws_manager.send(subscribe_msg)
    if send_error:
        ws_manager.close()
        return {"error": f"Failed to subscribe: {send_error}"}

    collected_messages = []
    end_time = time.time() + duration

    # Loop until duration expires or we hit max_messages
    while time.time() < end_time and len(collected_messages) < max_messages:
        response = ws_manager.receive(timeout=0.5)  # non-blocking small timeout
        if response:
            try:
                msg_data = json.loads(response)
                # rosbridge subscription responses include "msg" field
                if "msg" in msg_data:
                    collected_messages.append(msg_data["msg"])
            except json.JSONDecodeError:
                # skip malformed data
                continue

    # Unsubscribe when done
    unsubscribe_msg = {
        "op": "unsubscribe",
        "topic": topic_name
    }
    ws_manager.send(unsubscribe_msg)
    ws_manager.close()

    return {
        "topic": topic_name,
        "collected_count": len(collected_messages),
        "messages": collected_messages
    }


@mcp.tool()
def publish_for_durations(topic: str = "", msg_type: str = "", messages: list = None, durations: list = None) -> dict:
    """
    Publish a sequence of messages to a given ROS topic with delays in between.

    Args:
        topic (str): ROS topic name (e.g., "/cmd_vel")
        msg_type (str): ROS message type (e.g., "geometry_msgs/Twist")
        messages (list): A list of message dictionaries (ROS-compatible payloads)
        durations (list): A list of durations (seconds) to wait between messages

    Returns:
        dict:
            {
                "success": True,
                "published_count": <number of messages>,
                "topic": topic,
                "msg_type": msg_type
            }
            OR {"error": "<error message>"} if something failed
    """
    # Validate critical args before publishing
    if not topic or not msg_type or messages is None or durations is None:
        return {"error": "Missing required arguments: topic, msg_type, messages, and durations must all be provided."}

    # Ensure same length for messages & durations
    if len(messages) != len(durations):
        return {"error": "messages and durations must have the same length"}

    # Iterate and publish each message with a delay
    for i, (msg, delay) in enumerate(zip(messages, durations)):
        # Build the rosbridge publish message
        publish_msg = {"op": "publish", "topic": topic, "msg": msg}

        # Send it
        send_error = ws_manager.send(publish_msg)
        if send_error:
            ws_manager.close()
            return {"error": f"Failed at message {i + 1}: {send_error}", "published_count": i}

        # Wait before sending the next message
        time.sleep(delay)

    # Close after the full sequence
    ws_manager.close()

    return {"success": True, "published_count": len(messages), "topic": topic, "msg_type": msg_type}


if __name__ == "__main__":
    mcp.run(transport="stdio")
