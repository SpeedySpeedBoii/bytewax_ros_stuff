from typing import Callable, Generator, List, Optional, Tuple, TypeVar, cast
import bytewax.operators as op
from bytewax.dataflow import Stream
from bytewax.operators import KeyedStream
from transitions import Machine


T = TypeVar("T")
U = TypeVar("U")


def create_keyed_stream(stream: Stream[T]) -> KeyedStream[T]:
    return op.key_on(f"k_{stream.stream_id.split('.')[-2]}", stream, lambda _: "key")


def state_machine_stream(
    step_id: str,
    up: KeyedStream[T],
    machine: Machine,
) -> KeyedStream[str]:

    def mapper(input_item: T) -> str:
        machine.tick(input_item)
        return cast(str, machine.state)

    return op.map_value(step_id, up, mapper)
