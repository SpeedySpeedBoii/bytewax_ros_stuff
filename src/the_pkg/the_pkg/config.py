from dataclasses import dataclass
import math


@dataclass
class Point:
    x: float
    y: float

    def distance_from(self, other: "Point") -> float:
        horizontal_distance = self.x - other.x
        vertical_distance = self.y - other.y
        return math.sqrt(((horizontal_distance**2) + (vertical_distance**2)))


@dataclass
class Config:
    home_position: Point
    home_radius: float
    rtl_alt: float
    rtl_alt_tolerance: float


default_config = Config(
    home_position=Point(100, 100), home_radius=10, rtl_alt=100, rtl_alt_tolerance=10
)
