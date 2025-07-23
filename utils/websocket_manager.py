import socket
import json
import websocket._core as websocket
import base64
from typing import Optional

class WebSocketManager:
    def __init__(self, ip: str, port: int, local_ip: str):
        self.ip = ip
        self.port = port
        self.local_ip = local_ip
        self.ws = None

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


    def receive(self, timeout: float = 2.0) -> Optional[str]:
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
            message (dict): The Rosbridge message dictionary to send
            timeout (float): Seconds to wait for a response. Default = 2.0.

        Returns:
            dict: The parsed JSON response,
                  OR {"error": "<error message>"} if timeout/error occurred,
                  OR {"error": "invalid_json", "raw": <response>} if decoding fails.
        """
        self.connect()
        if self.ws:
            try:
                self.send(message)
                response = self.receive(timeout=timeout)

                if response is None or response.strip() == "":
                    return {"error": "no response or timeout from rosbridge"}

                try:
                    data = json.loads(response)
                    return data
                except json.JSONDecodeError as e:
                    print(f"[WebSocket] JSON decode error: {e}")
                    return {"error": "invalid_json", "raw": response}

            except Exception as e:
                print(f"[WebSocket] Request error: {e}")
                return {"error": str(e)}

        return {"error": "not connected to rosbridge"}

    def close(self):
        if self.ws and self.ws.connected:
            try:
                self.ws.close()
                print("[WebSocket] Closed")
            except Exception as e:
                print(f"[WebSocket] Close error: {e}")
            finally:
                self.ws = None
