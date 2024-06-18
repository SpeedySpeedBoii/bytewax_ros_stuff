from the_pkg.flow.rtl_validator import create_rtl_validator_sm


def main() -> None:
    machine = create_rtl_validator_sm()
    machine.get_combined_graph().draw("sm.png", prog="dot")


if __name__ == "__main__":
    main()
