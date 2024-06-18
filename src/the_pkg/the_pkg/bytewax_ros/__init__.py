from functools import partial
from types import TracebackType
from threading import Thread
from typing import Type, TypeVar
from bytewax.dataflow import Dataflow
from bytewax.dataflow import Stream
import bytewax.operators as op
import rclpy
from rclpy.task import Future
from rclpy.node import Node
from .connectors import SubscriptionSource, TopicData

__all__ = ["RosConnectorFactory", "TopicData"]

T = TypeVar("T")


class RosConnectorFactory:

    def __init__(self, *, node_name: str):
        self._node_name = node_name
        self._node: Node
        self._thread: Thread
        self._stop_future: Future

    def create_topic_input(
        self, flow: Dataflow, topic: str, message_type: Type[T]
    ) -> Stream[TopicData[T]]:
        return op.input(topic, flow, SubscriptionSource(self._node, topic, message_type))

    def __enter__(self) -> "RosConnectorFactory":
        rclpy.init()
        self._node = Node(self._node_name)
        self._stop_future = Future(executor=rclpy.get_global_executor())  # type: ignore
        self._thread = Thread(
            target=partial(rclpy.spin_until_future_complete, self._node, self._stop_future)
        )
        self._thread.start()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        self._node.destroy_node()  # type: ignore
        self._stop_future.set_result(None)  # type: ignore
        self._thread.join()
        rclpy.shutdown()
