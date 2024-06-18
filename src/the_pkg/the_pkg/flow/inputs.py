from dataclasses import dataclass
from bytewax.dataflow import Dataflow, Stream
from bytewax.operators import KeyedStream
from geometry_msgs.msg import Point as PointMsg  # type: ignore
from std_msgs.msg import Int8, Float32  # type: ignore
from the_pkg.bytewax_ros import TopicData
from the_pkg.bytewax_ros import RosConnectorFactory
from the_pkg.bytewax_utils import create_keyed_stream


@dataclass
class InputStreams:
    position: Stream[TopicData[PointMsg]]
    heartbeat: Stream[TopicData[Int8]]
    height: Stream[TopicData[Float32]]

    k_position: KeyedStream[TopicData[PointMsg]]
    k_heartbeat: KeyedStream[TopicData[Int8]]
    k_height: KeyedStream[TopicData[Float32]]


def create_inputs(ros_connector_factory: RosConnectorFactory, flow: Dataflow) -> InputStreams:
    position = ros_connector_factory.create_topic_input(flow, "/position", PointMsg)
    heartbeat = ros_connector_factory.create_topic_input(flow, "/heartbeat", Int8)
    height = ros_connector_factory.create_topic_input(flow, "/height", Float32)

    k_position = create_keyed_stream(position)
    k_heartbeat = create_keyed_stream(heartbeat)
    k_height = create_keyed_stream(height)

    return InputStreams(position, heartbeat, height, k_position, k_heartbeat, k_height)
