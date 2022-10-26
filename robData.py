from dataclasses import dataclass
from typing import List, Tuple, Dict, TYPE_CHECKING
from intersection import Intersection
from directions import Direction
if TYPE_CHECKING:
    from intention import Intention

@dataclass
class RobData:
    history: list
    pmap: List[Tuple[int, int]]
    intersections: Dict[Tuple[int, int], Intersection]
    current_intersection: Intersection
    starting_position: Tuple[int, int]
    path: List[Intersection]
    intersections_intentions: List['Intention']