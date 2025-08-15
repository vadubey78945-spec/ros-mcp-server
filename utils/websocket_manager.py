import json
from typing import Optional, Union

import websocket


def parse_json(raw: Optional[Union[str, bytes]]) -> Optional[dict]:
    """
    Safely parse JSON from string or bytes.
    
    Args:
        raw: JSON string, bytes, or None
        
    Returns:
        Parsed dict if successful, None if raw is None or parsing fails
    """
    if raw is None:
        return None
    if isinstance(raw, bytes):
        raw = raw.decode("utf-8", errors="replace")
    try:
        return json.loads(raw)
    except (json.JSONDecodeError, TypeError):
        return None


class WebSocketManager:
    def __init__(self, ip: str, port: int):
        self.ip = ip
        self.port = port
        self.ws = None

    def set_ip(self, ip: str, port: int):
        """
        Set the IP and port for the WebSocket connection.
        """
        self.ip = ip
        self.port = port
        print(f"[WebSocket] IP set to {self.ip}:{self.port}")

    def connect(self) -> Optional[str]:
        """
        Attempt to establish a WebSocket connection.

        Returns:
            None if successful,
            or an error message string if connection failed.
        """
        if self.ws is None or not self.ws.connected:
            try:
                url = f"ws://{self.ip}:{self.port}"
                self.ws = websocket.create_connection(url, timeout=2.0)
                print("[WebSocket] Connected (2s timeout)")
                return None  # no error
            except Exception as e:
                error_msg = f"[WebSocket] Connection error: {e}"
                print(error_msg)
                self.ws = None
                return error_msg
        return None  # already connected, no error

    def send(self, message: dict) -> Optional[str]:
        """
        Send a JSON-serializable message over WebSocket.

        Returns:
            None if successful,
            or an error message string if send failed.
        """
        conn_error = self.connect()
        if conn_error:
            return conn_error  # failed to connect

        if self.ws:
            try:
                json_msg = json.dumps(message)  # ensure it's JSON-serializable
                self.ws.send(json_msg)
                return None  # no error
            except TypeError as e:
                error_msg = f"[WebSocket] JSON serialization error: {e}"
                print(error_msg)
                self.close()
                return error_msg
            except Exception as e:
                error_msg = f"[WebSocket] Send error: {e}"
                print(error_msg)
                self.close()
                return error_msg

        return "[WebSocket] Not connected, send aborted."

    def receive(self, timeout: float = 2.0) -> Optional[Union[str, bytes]]:
        """
        Receive a single message from rosbridge within the given timeout.

        Args:
            timeout (float): Seconds to wait before timing out. Default = 2.0.

        Returns:
            Optional[str]: JSON string received from rosbridge, or None if timeout/error.
        """
        self.connect()
        if self.ws:
            try:
                # Temporarily set the receive timeout
                self.ws.settimeout(timeout)
                raw = self.ws.recv()  # rosbridge sends JSON as a string
                return raw
            except Exception as e:
                print(f"[WebSocket] Receive error or timeout: {e}")
                self.close()
                return None
        return None

    def request(self, message: dict, timeout: float = 2.0) -> dict:
        """
        Send a request to Rosbridge and return the response.

        Args:
            message (dict): The Rosbridge message dictionary to send.
            timeout (float): Seconds to wait for a response. Default = 2.0.

        Returns:
            dict:
                - Parsed JSON response if successful.
                - {"error": "<error message>"} if connection/send/receive fails.
                - {"error": "invalid_json", "raw": <response>} if decoding fails.
        """
        # Attempt connection
        conn_error = self.connect()
        if conn_error:
            return {"error": conn_error}

        # Attempt to send the message
        send_error = self.send(message)
        if send_error:
            return {"error": send_error}

        # Attempt to receive a response
        response = self.receive(timeout=timeout)
        if response is None:
            return {"error": "no response or timeout from rosbridge"}

        # Attempt to parse JSON
        parsed_response = parse_json(response)
        if parsed_response is None:
            print(f"[WebSocket] JSON decode error for response: {response}")
            return {"error": "invalid_json", "raw": response}
        return parsed_response

    def close(self):
        if self.ws and self.ws.connected:
            try:
                self.ws.close()
                print("[WebSocket] Closed")
            except Exception as e:
                print(f"[WebSocket] Close error: {e}")
            finally:
                self.ws = None
