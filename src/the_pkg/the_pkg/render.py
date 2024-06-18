from unittest.mock import Mock
import bytewax.operators as op
from bytewax.visualize import to_mermaid
from bytewax.testing import TestingSource
from the_pkg.bytewax_ros import RosConnectorFactory
from the_pkg.flow.flow import create_flow
from the_pkg.config import default_config
from the_pkg.flow.rtl_validator import create_rtl_validator_sm


def create_ros_connector_factory_mock() -> Mock:
    mock = Mock(spec=RosConnectorFactory)
    mock.create_topic_input.side_effect = lambda flow, topic, _: op.input(
        topic, flow, TestingSource([])
    )
    return mock


def main() -> None:
    machine = create_rtl_validator_sm()
    machine.get_combined_graph().draw("sm.png", prog="dot")

    flow = create_flow(create_ros_connector_factory_mock(), default_config)
    s = to_mermaid(flow)
    with open("flow.md", "w", encoding="utf8") as f:
        f.write("```mermaid\n")
        f.write(s)
        f.write("```")


if __name__ == "__main__":
    main()
