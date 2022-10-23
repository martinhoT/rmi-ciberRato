from typing import List, Tuple, Dict
from intersection import Intersection
from directions import Direction

@dataclass
class RobState:
    history: list
    pmap: List[Tuple[int, int]]
    intersections: Dict[Intersection, Direction]
    current_intersection: Intersection
    starting_position: Tuple[int, int]