import socket
import json
import websocket
import base64

class WebSocketManager:
    def __init__(self, ip: str, port: int, local_ip: str):
        self.ip = ip
        self.port = port
        self.local_ip = local_ip
        self.ws = None

    def connect(self):
        if self.ws is None or not self.ws.connected:
            sock = socket.create_connection((self.ip, self.port), source_address=(self.local_ip, 0))
            ws = websocket.WebSocket()
            ws.sock = sock
            ws.connect(f"ws://{self.ip}:{self.port}")
            self.ws = ws
            print("[WebSocket] Connected")

    def send(self, message: dict):
        self.connect()
        if self.ws:
            try:
                # Ensure message is JSON serializable
                json_msg = json.dumps(message)
                self.ws.send(json_msg)
            except TypeError as e:
                print(f"[WebSocket] JSON serialization error: {e}")
                self.close()
            except Exception as e:
                print(f"[WebSocket] Send error: {e}")
                self.close()


    def receive_binary(self) -> bytes:
        self.connect()
        if self.ws:
            try:
                raw = self.ws.recv()  # raw is JSON string (type: str)
                return raw
            except Exception as e:
                print(f"Receive error: {e}")
                self.close()
        return b""


    def close(self):
        if self.ws and self.ws.connected:
            try:
                self.ws.close()
                print("[WebSocket] Closed")
            except Exception as e:
                print(f"[WebSocket] Close error: {e}")
            finally:
                self.ws = None
