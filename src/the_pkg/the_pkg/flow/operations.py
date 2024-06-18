from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from math import isclose
import time
from typing import Optional, Tuple
import bytewax.operators as op
import bytewax.operators.windowing as win
from bytewax.dataflow import Stream
from bytewax.operators import KeyedStream
from geometry_msgs.msg import Point as PointMsg  # type: ignore
from std_msgs.msg import Int8, Float32  # type: ignore
from the_pkg.bytewax_ros import TopicData
from the_pkg.config import Config
from the_pkg.config import Point


def is_in_rtl_stream(heartbeat: KeyedStream[TopicData[Int8]]) -> KeyedStream[bool]:
    return op.map_value("is_in_rtl", heartbeat, lambda x: x.message.data == 3)


def is_position_up_to_date_stream(position: KeyedStream[TopicData[PointMsg]]) -> KeyedStream[bool]:
    return op.map_value(
        "is_position_up_to_date",
        position,
        lambda x: time.monotonic() - x.message_timestamp < 5,
    )


def is_near_home_stream(
    config: Config, position: KeyedStream[TopicData[PointMsg]]
) -> KeyedStream[bool]:
    return op.map_value(
        "is_near_home",
        position,
        lambda message: Point(message.x, message.y).distance_from(  # type: ignore
            config.home_position
        )
        < config.home_radius,
    )


def gap_between_last_2_heartbeats_stream(
    heartbeat: KeyedStream[TopicData[Int8]],
) -> KeyedStream[float]:
    @dataclass
    class MapperContext:
        last_heartbeat_timestamp: float
        gap: float

    def mapper(
        context: Optional[MapperContext], heartbeat: TopicData[Int8]
    ) -> Tuple[MapperContext, float]:
        if context is None:
            return MapperContext(heartbeat.message_timestamp, 0), 0
        elif heartbeat.message_timestamp == context.last_heartbeat_timestamp:
            return context, context.gap
        else:
            gap = heartbeat.message_timestamp - context.last_heartbeat_timestamp
            return MapperContext(heartbeat.message_timestamp, gap), gap

    return op.stateful_map("gap_between_last_2_heartbeats", heartbeat, mapper)


def is_near_rtl_alt_stream(
    config: Config, height: KeyedStream[TopicData[Float32]]
) -> KeyedStream[bool]:
    return op.map_value(
        "is_near_rtl_alt",
        height,
        lambda m: isclose(m.message.data, config.rtl_alt, abs_tol=config.rtl_alt_tolerance),
    )


def vertical_velocity_stream(
    config: Config, height: KeyedStream[TopicData[Float32]]
) -> KeyedStream[Float32]:
    clock = win.SystemClock()
    windower = win.SlidingWindower(
        timedelta(seconds=5),
        timedelta(seconds=1),
        align_to=datetime(2022, 1, 1, tzinfo=timezone.utc),
    )
    last_heights = win.collect_window("last_heights", height, clock, windower).down
    return op.map_value(
        "vertical_velocity",
        last_heights,
        lambda heights: (heights[1][-1].message.data - heights[1][0].message.data) / 5,
    )


def is_rising_up_stream(vertical_velocity: KeyedStream[Float32]) -> KeyedStream[bool]:
    return op.map_value("is_rising_up", vertical_velocity, lambda v: v > 10)
