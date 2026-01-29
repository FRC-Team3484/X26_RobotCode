from wpilib import Color, AddressableLED
from wpimath.units import meters_per_second
from frc3484.leds import ColorStack, ColorWave, FallingSand, Fire

from enum import Enum

class PatternState(Enum):
    fill = 0,
    empty = 1

