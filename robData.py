from dataclasses import dataclass, field
from typing import Callable, List, Tuple, Dict, TYPE_CHECKING
from graph import Checkpoint, Intersection
if TYPE_CHECKING:
    from intention import Intention

@dataclass
class RobData:
    pmap:                       List[Tuple[int, int]]               = field(default_factory=list)
    intersections:              Dict[Tuple[int, int], Intersection] = field(default_factory=dict)
    previous_node:              Intersection                        = field(default=None)
    starting_position:          Tuple[int, int]                     = field(default=())
    path:                       List[Intersection]                  = field(default_factory=list)
    intersections_intentions:   List['Intention']                   = field(default_factory=list)
    checkpoints:                Dict[int, Checkpoint]               = field(default_factory=dict)
    discontinuities:            int                                 = field(default=0)
    finish_condition:           Callable[['RobData'], bool]         = field(default=lambda _: False)

    def finished(self) -> bool:
        """When the robot data suggests that the challenge has been finished. Dependent on the provided finish condition."""
        return self.finish_condition(self)