from enum import Enum

class Direction(Enum):
    W = 0
    N = 1
    E = 2
    S = 3

# Has to be ordered
DIRECTIONS_ARRAY = [ Direction.W, Direction.N, Direction.E, Direction.S ]

def opposite_direction(direction: Direction) -> Direction:
    return DIRECTIONS_ARRAY[(direction.value - 2) % 4]

def left_direction(direction: Direction) -> Direction:
    return DIRECTIONS_ARRAY[(direction.value - 1) % 4]

def right_direction(direction: Direction) -> Direction:
    return DIRECTIONS_ARRAY[(direction.value + 1) % 4]

# Left
# DIRECTIONS_ARRAY[ (direction.value - 1) % 4 ]
# Right
# DIRECTIONS_ARRAY[ (direction.value + 1) % 4 ]