import socket
import json
import websocket

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
                self.ws.send(json.dumps(message))
            except Exception as e:
                print(f"[WebSocket] Send error: {e}")
                self.close()

    def close(self):
        if self.ws and self.ws.connected:
            try:
                self.ws.close()
                print("[WebSocket] Closed")
            except Exception as e:
                print(f"[WebSocket] Close error: {e}")
            finally:
                self.ws = None
