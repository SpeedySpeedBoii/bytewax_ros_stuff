```mermaid
flowchart TD
subgraph "the_flow (Dataflow)"
the_flow./position["/position (input)"]
the_flow./heartbeat["/heartbeat (input)"]
the_flow./height["/height (input)"]
the_flow.k_/position["k_/position (key_on)"]
the_flow./position -- "down → up" --> the_flow.k_/position
the_flow.k_/heartbeat["k_/heartbeat (key_on)"]
the_flow./heartbeat -- "down → up" --> the_flow.k_/heartbeat
the_flow.k_/height["k_/height (key_on)"]
the_flow./height -- "down → up" --> the_flow.k_/height
the_flow.is_near_home["is_near_home (map_value)"]
the_flow.k_/position -- "down → up" --> the_flow.is_near_home
the_flow.is_position_up_to_date["is_position_up_to_date (map_value)"]
the_flow.k_/position -- "down → up" --> the_flow.is_position_up_to_date
the_flow.is_in_rtl["is_in_rtl (map_value)"]
the_flow.k_/heartbeat -- "down → up" --> the_flow.is_in_rtl
the_flow.k_gap_between_last_2_heartbeats["k_gap_between_last_2_heartbeats (stateful_map)"]
the_flow.k_/heartbeat -- "down → up" --> the_flow.k_gap_between_last_2_heartbeats
the_flow.is_near_rtl_alt["is_near_rtl_alt (map_value)"]
the_flow.k_/height -- "down → up" --> the_flow.is_near_rtl_alt
the_flow.last_heights["last_heights (collect_window)"]
the_flow.k_/height -- "down → up" --> the_flow.last_heights
the_flow.vertical_velocity["vertical_velocity (map_value)"]
the_flow.last_heights -- "down → up" --> the_flow.vertical_velocity
the_flow.is_rising_up["is_rising_up (map_value)"]
the_flow.vertical_velocity -- "down → up" --> the_flow.is_rising_up
the_flow.rtl_validator_input_tuple["rtl_validator_input_tuple (join)"]
the_flow.is_near_home -- "down → sides" --> the_flow.rtl_validator_input_tuple
the_flow.is_position_up_to_date -- "down → sides" --> the_flow.rtl_validator_input_tuple
the_flow.is_in_rtl -- "down → sides" --> the_flow.rtl_validator_input_tuple
the_flow.k_gap_between_last_2_heartbeats -- "down → sides" --> the_flow.rtl_validator_input_tuple
the_flow.is_near_rtl_alt -- "down → sides" --> the_flow.rtl_validator_input_tuple
the_flow.is_rising_up -- "down → sides" --> the_flow.rtl_validator_input_tuple
the_flow.rtl_validator_input["rtl_validator_input (map_value)"]
the_flow.rtl_validator_input_tuple -- "down → up" --> the_flow.rtl_validator_input
the_flow.rtl_validator["rtl_validator (map_value)"]
the_flow.rtl_validator_input -- "down → up" --> the_flow.rtl_validator
the_flow.i["i (inspect)"]
the_flow.rtl_validator -- "down → up" --> the_flow.i
end```