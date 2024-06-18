from dataclasses import dataclass
import time
from typing import Generic, Optional, Type, TypeVar
from datetime import timedelta
from rclpy.node import Node
from bytewax.inputs import SimplePollingSource

T = TypeVar("T")


@dataclass
class TopicData(Generic[T]):
    message: T
    message_timestamp: float


class SubscriptionSource(SimplePollingSource[TopicData[T]]):
    def __init__(self, node: Node, topic: str, message_type: Type[T]):
        super().__init__(timedelta(milliseconds=100))
        self._subscription = node.create_subscription(
            message_type, topic, self._on_new_message, qos_profile=10
        )
        self._topic_data: Optional[TopicData[T]] = None

    def next_item(self) -> TopicData[T]:
        return self._topic_data  # type: ignore

    def _on_new_message(self, message: T) -> None:
        self._topic_data = TopicData(message=message, message_timestamp=time.monotonic())
