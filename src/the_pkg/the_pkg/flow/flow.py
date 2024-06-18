from dataclasses import dataclass
from typing import Callable, Generator, List
from transitions.extensions import HierarchicalGraphMachine
import bytewax.operators as op
from bytewax.dataflow import Dataflow
from transitions import Machine
from the_pkg.config import Config
from the_pkg.bytewax_ros import RosConnectorFactory
from the_pkg.bytewax_utils import state_machine_stream
from .rtl_validator import create_rtl_validator_sm, RtlValidatorInput
from .inputs import create_inputs
from .operations import (
    is_near_rtl_alt_stream,
    is_near_home_stream,
    is_position_up_to_date_stream,
    is_in_rtl_stream,
    gap_between_last_2_heartbeats_stream,
    is_rising_up_stream,
    vertical_velocity_stream,
)


def create_flow(ros_connector_factory: RosConnectorFactory, config: Config) -> Dataflow:
    flow = Dataflow("the_flow")
    inputs = create_inputs(ros_connector_factory, flow)

    is_near_home = is_near_home_stream(config, inputs.k_position)
    is_position_up_to_date = is_position_up_to_date_stream(inputs.k_position)
    is_in_rtl = is_in_rtl_stream(inputs.k_heartbeat)
    gap_between_last_2_heartbeats = gap_between_last_2_heartbeats_stream(inputs.k_heartbeat)
    is_near_rtl_alt = is_near_rtl_alt_stream(config, inputs.k_height)
    vertical_velocity = vertical_velocity_stream(config, inputs.k_height)
    is_rising_up = is_rising_up_stream(vertical_velocity)

    s_rtl_validator_input_tuple = op.join(
        "rtl_validator_input_tuple",
        is_near_home,
        is_position_up_to_date,
        is_in_rtl,
        gap_between_last_2_heartbeats,
        is_near_rtl_alt,
        is_rising_up,
        emit_mode="running",
    )
    s_rtl_validator_input = op.map_value(
        "rtl_validator_input",
        s_rtl_validator_input_tuple,
        lambda t: RtlValidatorInput(
            *t,  # type: ignore
        ),
    )

    machine = create_rtl_validator_sm()

    s_rtl_validator = state_machine_stream("rtl_validator", s_rtl_validator_input, machine)
    op.inspect("i", s_rtl_validator)

    return flow


"""
- entered rtl (from heartbeat)
- as long as we are not close to home
    - if (the pos is not relevant) or (the previous heartbeat message is too old):
        - wait for stop
        - wait for raise up
        - check raise up until RTL_ALT
    else:
        - calculate "real rtl alt"
        - wait for stop
        - if not in rtl alt
            - wait for raise up
            - check raise up until "real rtl alt"
            - wait_for raise up to stop
        - wait for fly to home
        - validate pitch and yaw
"""
