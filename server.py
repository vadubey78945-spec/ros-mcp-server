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


@mcp.tool(description=("Fetch available topics from the ROS bridge.\nExample:\nget_topics()"))
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


@mcp.tool(
    description=("Get the message type for a specific topic.\nExample:\nget_topic_type('/cmd_vel')")
)
def get_topic_type(topic: str) -> dict:
    """
    Get the message type for a specific topic.

    Args:
        topic (str): The topic name (e.g., '/cmd_vel')

    Returns:
        dict: Contains the 'type' field with the message type,
            or an error message if topic doesn't exist.
    """
    # rosbridge service call to get topic type
    message = {
        "op": "call_service",
        "service": "/rosapi/topic_type",
        "type": "rosapi/TopicType",
        "args": {"topic": topic},
        "id": f"get_topic_type_request_{topic.replace('/', '_')}",
    }

    # Request topic type from rosbridge
    response = ws_manager.request(message)
    ws_manager.close()

    # Return topic type if present
    if response and "values" in response:
        topic_type = response["values"].get("type", "")
        if topic_type:
            return {"topic": topic, "type": topic_type}
        else:
            return {"error": f"Topic {topic} does not exist or has no type"}
    else:
        return {"error": f"Failed to get type for topic {topic}"}


@mcp.tool(
    description=(
        "Get the complete structure/definition of a message type.\n"
        "Example:\n"
        "get_message_details('geometry_msgs/Twist')"
    )
)
def get_message_details(message_type: str) -> dict:
    """
    Get the complete structure/definition of a message type.

    Args:
        message_type (str): The message type (e.g., 'geometry_msgs/Twist')

    Returns:
        dict: Contains the message structure with field names and types,
            or an error message if the message type doesn't exist.
    """
    # rosbridge service call to get message details
    message = {
        "op": "call_service",
        "service": "/rosapi/message_details",
        "type": "rosapi/MessageDetails",
        "args": {"type": message_type},
        "id": f"get_message_details_request_{message_type.replace('/', '_')}",
    }

    # Request message details from rosbridge
    response = ws_manager.request(message)
    ws_manager.close()

    # Return message structure if present
    if response and "values" in response:
        typedefs = response["values"].get("typedefs", [])
        if typedefs:
            # Parse the structure into a more readable format
            structure = {}
            for typedef in typedefs:
                type_name = typedef.get("type", message_type)
                field_names = typedef.get("fieldnames", [])
                field_types = typedef.get("fieldtypes", [])

                fields = {}
                for name, ftype in zip(field_names, field_types):
                    fields[name] = ftype

                structure[type_name] = {"fields": fields, "field_count": len(fields)}

            return {"message_type": message_type, "structure": structure}
        else:
            return {"error": f"Message type {message_type} not found or has no definition"}
    else:
        return {"error": f"Failed to get details for message type {message_type}"}


@mcp.tool(
    description=(
        "Get list of nodes that are publishing to a specific topic.\n"
        "Example:\n"
        "get_publishers_for_topic('/cmd_vel')"
    )
)
def get_publishers_for_topic(topic: str) -> dict:
    """
    Get list of nodes that are publishing to a specific topic.

    Args:
        topic (str): The topic name (e.g., '/cmd_vel')

    Returns:
        dict: Contains list of publisher node names,
            or a message if no publishers found.
    """
    # rosbridge service call to get publishers
    message = {
        "op": "call_service",
        "service": "/rosapi/publishers",
        "type": "rosapi/Publishers",
        "args": {"topic": topic},
        "id": f"get_publishers_for_topic_request_{topic.replace('/', '_')}",
    }

    # Request publishers from rosbridge
    response = ws_manager.request(message)
    ws_manager.close()

    # Return publishers if present
    if response and "values" in response:
        publishers = response["values"].get("publishers", [])
        return {"topic": topic, "publishers": publishers, "publisher_count": len(publishers)}
    else:
        return {"error": f"Failed to get publishers for topic {topic}"}


@mcp.tool(
    description=(
        "Get list of nodes that are subscribed to a specific topic.\n"
        "Example:\n"
        "get_subscribers_for_topic('/cmd_vel')"
    )
)
def get_subscribers_for_topic(topic: str) -> dict:
    """
    Get list of nodes that are subscribed to a specific topic.

    Args:
        topic (str): The topic name (e.g., '/cmd_vel')

    Returns:
        dict: Contains list of subscriber node names,
            or a message if no subscribers found.
    """
    # rosbridge service call to get subscribers
    message = {
        "op": "call_service",
        "service": "/rosapi/subscribers",
        "type": "rosapi/Subscribers",
        "args": {"topic": topic},
        "id": f"get_subscribers_for_topic_request_{topic.replace('/', '_')}",
    }

    # Request subscribers from rosbridge
    response = ws_manager.request(message)
    ws_manager.close()

    # Return subscribers if present
    if response and "values" in response:
        subscribers = response["values"].get("subscribers", [])
        return {"topic": topic, "subscribers": subscribers, "subscriber_count": len(subscribers)}
    else:
        return {"error": f"Failed to get subscribers for topic {topic}"}


@mcp.tool(
    description=(
        "Subscribe to a ROS topic and return the first message received.\n"
        "Example:\n"
        "subscribe_once(topic='/cmd_vel', msg_type='geometry_msgs/msg/TwistStamped')"
    )
)
def subscribe_once(topic: str = "", msg_type: str = "") -> dict:
    """
    Subscribe to a given ROS topic via rosbridge and return the first message received.

    Args:
        topic (str): The ROS topic name (e.g., "/cmd_vel", "/joint_states").
        msg_type (str): The ROS message type (e.g., "geometry_msgs/Twist").

    Returns:
        dict:
            - {"msg": <parsed ROS message>} if successful
            - {"error": "<error message>"} if subscription or timeout fails
    """
    # Validate critical args before attempting subscription
    if not topic or not msg_type:
        return {"error": "Missing required arguments: topic and msg_type must be provided."}

    # Construct the rosbridge subscribe message
    subscribe_msg = {
        "op": "subscribe",
        "topic": topic,
        "type": msg_type,
        "queue_length": 1,  # request just one message
    }

    # Send subscription request & wait for a single response
    response = ws_manager.request(subscribe_msg)

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


@mcp.tool(
    description=(
        "Publish a single message to a ROS topic.\n"
        "Example:\n"
        "publish_once(topic='/cmd_vel', msg_type='geometry_msgs/msg/TwistStamped', msg={'linear': {'x': 1.0}})"
    )
)
def publish_once(topic: str = "", msg_type: str = "", msg: dict = {}) -> dict:
    """
    Publish a single message to a ROS topic via rosbridge.

    Args:
        topic (str): ROS topic name (e.g., "/cmd_vel")
        msg_type (str): ROS message type (e.g., "geometry_msgs/Twist")
        msg (dict): Message payload as a dictionary

    Returns:
        dict:
            - {"success": True} if sent without errors
            - {"error": "<error message>"} if connection/send failed
            - If rosbridge responds (usually it doesn’t for publish), parsed JSON or error info
    """
    # Validate critical args before attempting publish
    if not topic or not msg_type or msg == {}:
        return {
            "error": "Missing required arguments: topic, msg_type, and msg must all be provided."
        }

    # Construct rosbridge publish message
    publish_msg = {"op": "publish", "topic": topic, "msg": msg}

    # Send the message via ws_manager
    send_error = ws_manager.send(publish_msg)
    if send_error:
        ws_manager.close()
        return {"error": send_error}

    # rosbridge typically does NOT respond to publish requests
    # But we can still attempt to receive in case of errors
    response = ws_manager.receive()

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


@mcp.tool(
    description=(
        "Subscribe to a topic for a duration and collect messages.\n"
        "Example:\n"
        "subscribe_for_duration(topic='/cmd_vel', msg_type='geometry_msgs/msg/TwistStamped', duration=5, max_messages=10)"
    )
)
def subscribe_for_duration(
    topic: str = "", msg_type: str = "", duration: float = 5.0, max_messages: int = 100
) -> dict:
    """
    Subscribe to a ROS topic via rosbridge for a fixed duration and collect messages.

    Args:
        topic (str): ROS topic name (e.g. "/cmd_vel", "/joint_states")
        msg_type (str): ROS message type (e.g. "geometry_msgs/Twist")
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
    if not topic or not msg_type:
        return {"error": "Missing required arguments: topic and msg_type must be provided."}

    # Send subscription request
    subscribe_msg = {
        "op": "subscribe",
        "topic": topic,
        "type": msg_type,
        "queue_length": 10,  # allow some buffering
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
    unsubscribe_msg = {"op": "unsubscribe", "topic": topic}
    ws_manager.send(unsubscribe_msg)
    ws_manager.close()

    return {
        "topic": topic,
        "collected_count": len(collected_messages),
        "messages": collected_messages,
    }


@mcp.tool(
    description=(
        "Publish a sequence of messages with delays.\n"
        "Example:\n"
        "publish_for_durations(topic='/cmd_vel', msg_type='geometry_msgs/msg/TwistStamped', messages=[{'linear': {'x': 1.0}}, {'linear': {'x': 0.0}}], durations=[1, 2])"
    )
)
def publish_for_durations(
    topic: str = "", msg_type: str = "", messages: list = [], durations: list = []
) -> dict:
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
    if not topic or not msg_type or messages == [] or durations == []:
        return {
            "error": "Missing required arguments: topic, msg_type, messages, and durations must all be provided."
        }

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
