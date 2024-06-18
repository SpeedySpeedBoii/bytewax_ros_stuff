from dataclasses import dataclass
from transitions.extensions import HierarchicalGraphMachine


@dataclass
class RtlValidatorInput:
    is_near_home: bool
    is_position_up_to_date: bool
    is_in_rtl: bool
    gap_between_last_2_heartbeats: float
    is_near_rtl_alt: bool
    is_rising_up: bool


def create_rtl_validator_sm() -> HierarchicalGraphMachine:

    class RtlValidator(HierarchicalGraphMachine):
        def __init__(self) -> None:
            states = [
                "initial",
                "not rtl",
                {
                    "name": "rtl",
                    "initial": "validating",
                    "children": [
                        {
                            "name": "validating",
                            "initial": "start",
                            "children": [
                                "start",
                                {
                                    "name": "bad startup",
                                    'initial': 'wait for rise up',
                                    "children": [
                                        "wait for rise up",
                                        "rising up",
                                    ],
                                },
                                {
                                    "name": "good startup",
                                    "children": ["wait for rise up", "rising up"],
                                },
                                "wait fly home",
                                "flying home",
                            ],
                        },
                        "finished",
                        "bad rtl",
                    ],
                },
            ]
            super().__init__(self, states=states, initial="initial", show_conditions=True)
            self.add_transition("tick", "initial", "not rtl", conditions=self.not_in_rtl)
            self.add_transition(
                "tick", "not rtl", "rtl", conditions=self.is_in_rtl
            )
            self.add_transition("tick", "rtl", "not rtl", conditions=self.not_in_rtl)
            self.add_transition(
                "tick",
                "rtl_validating_start",
                "rtl_validating_bad startup",
                conditions=[self.position_is_up_to_date_and_no_heartbeat_gap],
            )

        def is_in_rtl(self, input_item: RtlValidatorInput) -> bool:
            return input_item.is_in_rtl

        def not_in_rtl(self, input_item: RtlValidatorInput) -> bool:
            return not input_item.is_in_rtl

        def position_is_up_to_date_and_no_heartbeat_gap(
            self, input_item: RtlValidatorInput
        ) -> bool:
            return (
                input_item.is_position_up_to_date and input_item.gap_between_last_2_heartbeats < 5
            )

    return RtlValidator()
