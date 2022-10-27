from dataclasses import dataclass, field
from typing import List, Tuple, Dict, TYPE_CHECKING
from intersection import Intersection
if TYPE_CHECKING:
    from intention import Intention

@dataclass
class RobData:
    # TODO: Is history still needed? (for C1 perhaps?)
    history:                    list                                = field(default_factory=list)
    pmap:                       List[Tuple[int, int]]               = field(default_factory=list)
    intersections:              Dict[Tuple[int, int], Intersection] = field(default_factory=dict)
    previous_intersection:      Intersection                        = field(default=None)
    starting_position:          Tuple[int, int]                     = field(default=())
    path:                       List[Intersection]                  = field(default_factory=list)
    intersections_intentions:   List['Intention']                   = field(default_factory=list)
    checkpoints:                Dict[int, Tuple[int, int]]          = field(default_factory=dict)