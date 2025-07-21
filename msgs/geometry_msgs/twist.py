from typing import List, Any, Protocol
import json

class Publisher(Protocol):
    def send(self, message: dict) -> None:
        ...

    def receive_binary(self) -> bytes:
        ...


def to_float(value: Any) -> float:
    try:
        return float(value)
    except (ValueError, TypeError):
        raise ValueError(f"Invalid float value: {value}")


class Twist:
    def __init__(self, publisher: Publisher, topic: str = "/cmd_vel"):
        self.publisher = publisher
        self.topic = topic

    def publish(self, linear: List[Any], angular: List[Any]):
        linear_f = [to_float(val) for val in linear]
        angular_f = [to_float(val) for val in angular]

        msg = {
            "op": "publish",
            "topic": self.topic,
            "msg": {
                "linear": {"x": linear_f[0], "y": 0, "z": 0},
                "angular": {"x": 0, "y": 0, "z": angular_f[2]}
            }
        }
        self.publisher.send(msg)
        
        return msg

    def publish_sequence(self, linear_seq: List[Any], angular_seq: List[Any], duration_seq: List[Any]):
        import time
        linear_flat = [to_float(val) for sublist in linear_seq for val in sublist]
        angular_flat = [to_float(val) for sublist in angular_seq for val in sublist]
        duration_f = [to_float(val) for val in duration_seq]

        for i in range(len(duration_f)):
            l = linear_flat[i*3:(i+1)*3]
            a = angular_flat[i*3:(i+1)*3]
            self.publish(l, a)
            time.sleep(duration_f[i])

    def subscribe(self, timeout: float = 2.0):
        """
        Subscribe to the Twist topic and return the latest message.

        Args:
            timeout (float): Timeout in seconds for receiving a message.

        Returns:
            str | None: JSON-formatted message data if available, otherwise None.
        """
        # Send a subscription request
        subscribe_msg = {
            "op": "subscribe",
            "topic": self.topic
        }
        self.publisher.send(subscribe_msg)

        # Wait for a message
        raw = self.publisher.receive_binary()
        if not raw:
            return None

        try:
            msg = json.loads(raw)
            if "msg" in msg:
                return json.dumps(msg["msg"], indent=2, ensure_ascii=False)
            return json.dumps(msg, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"[Twist] Failed to parse message: {e}")
            return None
