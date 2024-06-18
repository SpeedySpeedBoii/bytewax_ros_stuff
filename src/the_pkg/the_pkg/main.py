from bytewax.run import cli_main
from the_pkg.flow.flow import create_flow
from the_pkg.bytewax_ros import RosConnectorFactory
from the_pkg.config import default_config


def main() -> None:
    print("Hi from the_pkg.")
    with RosConnectorFactory(node_name="the_node") as bytewax_ros:
        cli_main(create_flow(bytewax_ros, default_config))  # type: ignore


if __name__ == "__main__":
    main()
