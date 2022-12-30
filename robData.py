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
    stages_conditions:          List[Callable[['RobData'], bool]]   = field(default_factory=lambda: [lambda _: False])
    stages:                     List['Intention']                   = field(default_factory=lambda: [None])
    previous_stage:             'Intention'                         = field(default=None)
    movement_guess:             MovementData                        = field(default_factory=MovementData)
    previous_action:            Tuple[float, float]                 = field(default=(0.0, 0.0))
    expected_noise:             float                               = field(default=0.0)

    def finished(self) -> bool:
        """When the robot data suggests that the challenge has been finished. Dependent on the provided stage conditions."""
        return self.stages_conditions[0](self) if self.stages_conditions else True
    
    def next_stage(self) -> 'Intention':
        """Pass on to the next stage, returning the intention to which the robot should transition."""
        if not self.stages_conditions or not self.stages:
            return None

        self.stages_conditions.pop(0)
        self.previous_stage = self.stages.pop(0)
        return self.previous_stage
