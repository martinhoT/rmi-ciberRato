from dataclasses import dataclass, field
from typing import Callable, List, Tuple, Dict, TYPE_CHECKING
from graph import Checkpoint, Intersection
if TYPE_CHECKING:
    from intention import Intention

@dataclass
class MovementData:
    out:            Tuple[float, float] = field(default=(0, 0))
    coordinates:    Tuple[float, float] = field(default=(0, 0))
    angle:          float               = field(default=0.0)

@dataclass
class RobData:
    pmap:                       List[Tuple[int, int]]               = field(default_factory=list)
    intersections:              Dict[Tuple[int, int], Intersection] = field(default_factory=dict)
    previous_intersection:      Intersection                        = field(default=None)
    starting_position:          Tuple[int, int]                     = field(default=())
    path:                       List[Intersection]                  = field(default_factory=list)
    intersections_intentions:   List['Intention']                   = field(default_factory=list)
    checkpoints:                Dict[int, Checkpoint]               = field(default_factory=dict)
    discontinuities:            int                                 = field(default=0)
    finish_condition:           Callable[['RobData'], bool]         = field(default=lambda _: False)
    prepare_before_finish:      bool                                = field(default=False)
    movement_guess:             MovementData                        = field(default_factory=MovementData)
    previous_action:            Tuple[float, float]                 = field(default=(0.0, 0.0))

    def finished(self) -> bool:
        """When the robot data suggests that the challenge has been finished. Dependent on the provided finish condition."""
        return self.finish_condition(self)
