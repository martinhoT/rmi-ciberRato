import random

from os import system
from typing import Dict, List, Set, Tuple, Sequence
from croblink import CMeasures
from graph import Checkpoint, Intersection, Node
from robData import RobData

from directions import Direction, left_direction, opposite_direction, right_direction
from utils import *


LOG = True
LOG_CLEAR = True
LOG_STARTING_POS = False
LOG_INTENTION = True
LOG_SENSORS = True
LOG_INTERSECTIONS = True
LOG_CALCULATED_PATH = True
LOG_GROUND = False
LOG_CHECKPOINTS = False
LOG_DISTANCE_KNOWN_INTERSECTION_AHEAD = True
LOG_MOVEMENT_GUESS = True
LOG_MAP = True

SPEED_OPTIMIZATIONS = True
MAX_SPEED = 0.15
SLOW_DOWN_PORTION = (1.0, 1.5)

class Intention:

    def __init__(self):
        self.velocity = 0.08

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        raise NotImplementedError()

    def log_measured(self, measures: CMeasures, rdata: RobData):
        if LOG:
            
            (x, y), position = self.obtain_position(measures, rdata)

            if LOG_CLEAR:
                system('clear')
            if LOG_STARTING_POS:
                print(rdata.starting_position)
            if LOG_INTENTION:
                print(self)
            if LOG_SENSORS:
                print(f'{measures.lineSensor} ({x:.3f}, {y:.3f}) -> ({position[0]:.3g}, {position[1]:.3g}) | {measures.compass} -> {get_direction(measures.compass)}')
            if LOG_MOVEMENT_GUESS:
                gx, gy = rdata.movement_guess.coordinates
                gangle = rdata.movement_guess.angle
                goutx, gouty = rdata.movement_guess.out
                print(f'Movement guess: Coords -> ({gx:.3f}, {gy:.3f}) | Angle -> {gangle:.1f} | Out -> ({goutx:.2f}, {gouty:.2f})')
            if LOG_INTERSECTIONS:
                print('Intersections:')
                for intersection_pos, intersection in rdata.intersections.items():
                    not_visited = intersection.get_possible_paths() - intersection.get_visited_paths()
                    if not_visited:
                        print(intersection_pos, "- Not Visited:" , not_visited, "- Neighbours:", intersection.get_neighbours())
                print('Previous intersection:', rdata.previous_intersection)
            if LOG_CALCULATED_PATH:
                print('Next intersections:', rdata.path)
                print('Next intentions:', rdata.intersections_intentions)
            if LOG_GROUND:
                print('Ground:', measures.ground)
            if LOG_CHECKPOINTS:
                print('Checkpoints:', rdata.checkpoints)
            if LOG_DISTANCE_KNOWN_INTERSECTION_AHEAD:
                print('Distance to known intersection ahead:', get_walkable_distance_to_closest_intersection_in_front_of_pos(
                    (x - rdata.starting_position[0], y - rdata.starting_position[1]),
                    get_direction(measures.compass),
                    rdata.intersections, rdata.pmap,
                    position))
            if LOG_MAP and rdata.pmap:
                for line in map_to_text(rdata.pmap, {i:checkpoint.get_coordinates() for i, checkpoint in rdata.checkpoints.items()}, rdata.intersections.keys()):
                    print(''.join(line))

    # The path is either a vertical or horizontal line
    def follow_path(self, measures: CMeasures) -> Tuple[int, int]:
        angle_to_track = get_angle_to_track(measures.compass)

        left = measures.lineSensor[:3].count("1")
        right = measures.lineSensor[4:].count("1")

        left_imbalance = left - right

        return (self.velocity * (1 - (-angle_to_track/45) - (left*1/3 if left_imbalance > 0 else 0)), 
                self.velocity * (1 - (angle_to_track/45) - (right*1/3 if left_imbalance < 0 else 0)))
    
    def update_neighbours(self, node: Node, rdata: RobData):
        if rdata.previous_intersection and node != rdata.previous_intersection:
            node.add_neighbour(rdata.previous_intersection)
            rdata.previous_intersection.add_neighbour(node)

    def line_sensor_discontinuity(self, lineSensor: List[str]) -> bool:
        """Report whether the current line sensors are likely to be heavily disrupted by noise."""
        groups = 1
        prev_sensor = lineSensor[0]
        for sensor in lineSensor[1:]:
            if sensor != prev_sensor:
                groups += 1
            prev_sensor = sensor
        
        return groups > 3

    def calculate_moves(self, direction: Direction, path: List[Node]) -> List['Intention']:
        
        moves = []
        for i in range(len(path) - 1):

            new_direction = get_direction_from_path(path[i], path[i+1])

            if direction == new_direction:
                move = MoveForward()
            else:
                move = Rotate(direction, new_direction)

            moves.append(move)
            direction = new_direction

        return moves
    
    def speed_up_func(self, x, max_speed, velocity, slow_down_portion) -> float:
        a, b = slow_down_portion
        
        if x < b:
            value = (max_speed - velocity)/velocity
            local_x = (x - a)/(b - a)

            if x < b - (b - a)/2:
                return 1 + value * 2**(20 * local_x - 11)
            return 1 + value * (1-2**(-20 * local_x + 9))

        return max_speed/velocity

    def obtain_position(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], Tuple[int, int]]:

        # Obtain position of robot in the map
        if measures.gpsReady:
            x, y = measures.x, measures.y
        else:
            x, y = rdata.movement_guess.coordinates
            x, y = estimate_pos(x, y, measures.compass, rdata.starting_position)
        position = round_pos(x, y, rdata.starting_position)
            
        return (x, y), position

    def set_route_to(self, key: Callable[[Node], bool], direction: Direction, start_node: Node, rdata: RobData):

        # Wavefront expansion to find the path to the closest node satisfying the key function
        path_to_closest = wavefront_expansion(start_node=start_node, key=key)

        if path_to_closest:
            path, _ = path_to_closest
            rdata.intersections_intentions = self.calculate_moves(direction, path)
            rdata.path = path[1:]

    def __str__(self): return self.__class__.__name__
    def __repr__(self): return str(self)


class Wander(Intention):

    def __init__(self, sample_loop: 'SampleLoop'=None):
        super().__init__()

        self.sample_loop = sample_loop

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        (x, y), position = self.obtain_position(measures, rdata)
        # Update the movement guess
        if not measures.gpsReady:
            rdata.movement_guess.coordinates = (x, y)
        
        if position not in rdata.pmap:
            rdata.pmap.append(position)

        direction = get_direction(measures.compass)

        # Robot is possibly off track or found a dead end
        n_active = measures.lineSensor.count("1")
        if n_active == 0 and not self.sample_loop:
            return (self.velocity, self.velocity), SampleLoop(Wander)

        # Robot is off track, add dead end to intersections
        if self.sample_loop and self.sample_loop.lineSensor.count("1") == 0:
            
            # Adjust position to the closest possible intersection
            intersection_pos = round_pos_to_intersection(x, y, rdata.starting_position)

            # If the intersection is not in the map, add it
            if intersection_pos not in rdata.intersections:

                intersection = self.create_intersection(intersection_pos, rdata.intersections)

                if rdata.previous_intersection:

                    intersection.add_visited_path(opposite_direction(direction))

                    # It's possible if finding dead-ends
                    if rdata.previous_intersection != intersection:
                        rdata.previous_intersection.add_visited_path(direction)

                    self.update_neighbours(intersection, rdata)

                else:

                    intersection.add_path(opposite_direction(direction))
                    
                rdata.previous_intersection = intersection

            return (0.0, 0.0), TurnBack()

        # When the robot data suggests that the stage has been finished
        if rdata.finished():
            return (0.0, 0.0), rdata.next_stage()

        # Line Sensors detected a discontinuity, likely disrupted by noise
        if self.line_sensor_discontinuity(measures.lineSensor):
            rdata.discontinuities += 1
            return (self.velocity / 5, self.velocity / 5), None

        # Save checkpoint if one was found
        if (measures.ground != -1) and measures.ground not in rdata.checkpoints:
            checkpoint_pos = round_pos_to_intersection(x, y, rdata.starting_position)
            checkpoint = Checkpoint(checkpoint_pos[0], checkpoint_pos[1], measures.ground)
            rdata.checkpoints[measures.ground] = checkpoint
     
        # Possible intersection found
        if self.check_if_intersection(measures.lineSensor):

            # Adjust position to the closest possible intersection
            line_sensor_x, line_sensor_y = get_line_sensor_pos(x, y, direction)
            intersection_pos = round_pos_to_intersection(line_sensor_x, line_sensor_y, rdata.starting_position)

            # Don't check for new intersections if the robot already has a path to follow
            if not rdata.intersections_intentions:

                # If the intersection is not in the map, add it
                if intersection_pos not in rdata.intersections:
                    self.create_intersection(intersection_pos, rdata.intersections)

                    return (0.0, 0.0), SampleLoop(CheckIntersectionForward, intersection_pos)

            # If robot has a path to follow and the intersection exists
            if intersection_pos in rdata.intersections:
                return None, TurnIntersection()
            
        velocity_modifier = 1
        if SPEED_OPTIMIZATIONS:
            closest = get_walkable_distance_to_closest_intersection_in_front_of_pos(
                position=(x - rdata.starting_position[0], y - rdata.starting_position[1]),
                direction=direction, intersections=rdata.intersections, pmap=rdata.pmap, rounded_position=position)

            if closest:
                closest_distance = closest[0]

                velocity_modifier = self.speed_up_func(closest_distance, MAX_SPEED, self.velocity, SLOW_DOWN_PORTION)

        action = self.follow_path(measures)
        return (action[0]*velocity_modifier, action[1]*velocity_modifier), Wander()
    
    def create_intersection(self, intersection_pos: Tuple[int, int],
            intersection_list: Dict[Tuple[int, int], Intersection]) -> Intersection:
        
        intersection = Intersection(intersection_pos[0], intersection_pos[1])
        intersection_list[intersection_pos] = intersection

        return intersection

    def check_if_intersection(self, lineSensor: List[str]) -> bool:

        # Robot is on track
        left = lineSensor[:4].count("1")
        right = lineSensor[3:].count("1")

        leftTurn = left >= 3 and lineSensor[0] == "1"
        rightTurn = right >= 3 and lineSensor[-1] == "1"

        return (leftTurn or rightTurn) and lineSensor[3] == '1'
        

class CheckIntersectionForward(Intention):

    def __init__(self, intersection_pos: Tuple[int, int], test_steps: int=7, sample_loop: 'SampleLoop'=None):
        super().__init__()
        self.intersection_pos = intersection_pos
        self.test_steps = test_steps
        self.found_directions = set()

        self.sample_loop = sample_loop

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        direction = get_direction(measures.compass)
        next_intention = None

        if self.sample_loop:
            
            if all(ls == '1' for ls in self.sample_loop.lineSensor[:3]):
                self.found_directions.add(left_direction(direction))

            if all(ls == '1' for ls in self.sample_loop.lineSensor[4:]):
                self.found_directions.add(right_direction(direction))

            # Intersection doesn't exist (noise)
            if not all(ls == '1' for ls in self.sample_loop.lineSensor[:3]) and \
                not all(ls == '1' for ls in self.sample_loop.lineSensor[4:]):

                (x, y), _ = self.obtain_position(measures, rdata)
                direction = get_direction(measures.compass)

                line_sensor_x, line_sensor_y = get_line_sensor_pos(x, y, direction)
                intersection = round_pos_to_intersection(line_sensor_x, line_sensor_y, rdata.starting_position)
                if intersection in rdata.intersections:
                    rdata.intersections.pop(intersection)

                return (self.velocity, self.velocity), Wander()

        else:

            if all(ls == '1' for ls in measures.lineSensor[:2]):
                self.found_directions.add(left_direction(direction))

            if all(ls == '1' for ls in measures.lineSensor[5:]):
                self.found_directions.add(right_direction(direction))

        if measures.lineSensor.count('0') >= 6:
            next_intention = CheckIntersectionForwardBacktrack(self.intersection_pos, self.found_directions)

        if self.test_steps == 0:
            next_intention = SampleLoop(CheckIntersectionForwardBacktrack, self.intersection_pos, self.found_directions)

        self.test_steps -= 1

        return (self.velocity, self.velocity), next_intention


class CheckIntersectionForwardBacktrack(Intention):

    def __init__(self, intersection_pos: Tuple[int, int], found_directions: Set[Direction], max_steps: int=10, sample_loop: 'SampleLoop'=None):
        super().__init__()
        self.steps = 0
        # NOTE: the maximum number of steps should not be too large. The robot should not leave the intersection, or else it will be lost
        self.max_steps = max_steps
        self.intersection_pos = intersection_pos
        self.found_directions = found_directions

        self.sample_loop = sample_loop

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        (x, y), _ = self.obtain_position(measures, rdata)
        direction = get_direction(measures.compass)
        
        # Check if path forward exists
        if self.sample_loop and direction not in self.found_directions and self.sample_loop.lineSensor.count('1') != 0:
            self.found_directions.add(direction)
        
        # Update the movement guess
        if not measures.gpsReady:
            rdata.movement_guess.coordinates = (x, y)

        line_sensor_pos = get_line_sensor_pos(x, y, direction)
        rounded_line_sensor_pos = round_pos(line_sensor_pos[0], line_sensor_pos[1], rdata.starting_position)
        closest = get_walkable_distance_to_closest_intersection_in_front_of_pos(
            position=line_sensor_pos, direction=direction, intersections=rdata.intersections.keys(), pmap=rdata.pmap, rounded_position=rounded_line_sensor_pos
        )
        intersection_in_front = closest[1] if closest is not None else None

        # If the robot is back at the intersection
        if self.steps == self.max_steps or intersection_in_front == self.intersection_pos:

            for found_direction in self.found_directions:
                rdata.intersections[self.intersection_pos].add_path( found_direction )
                
            rdata.intersections[self.intersection_pos].add_path( opposite_direction(direction) )
            
            return (0.0, 0.0), TurnIntersection() if intersection_in_front == self.intersection_pos else Wander()

        self.steps += 1

        return (-self.velocity, -self.velocity), None


class TurnIntersection(Intention):

    def __init__(self):
        super().__init__()

        # Whether or not the robot acted randomly in the last call. Can tell if the robot is lost
        self.acted_randomly = False

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        (x, y), _ = self.obtain_position(measures, rdata)
        direction = get_direction(measures.compass)

        line_sensor_x, line_sensor_y = get_line_sensor_pos(x, y, direction)
        intersection_pos = round_pos_to_intersection(line_sensor_x, line_sensor_y, rdata.starting_position)
        direction = get_direction(measures.compass)
        intersection = rdata.intersections[intersection_pos]

        if rdata.previous_intersection:
            intersection.add_visited_path(opposite_direction(direction))
            # It's possible if finding dead-ends
            if rdata.previous_intersection != intersection:
                rdata.previous_intersection.add_visited_path(direction)

        self.update_neighbours(intersection, rdata)

        # When the robot data suggests that the challenge has been finished
        if rdata.finished():
            return (0.0, 0.0), Wander()

        next_intention = None

        # If there are pre-calculated intentions, follow them
        if rdata.intersections_intentions:
            rdata.path.pop(0)
            next_intention = rdata.intersections_intentions.pop(0)
        
        # If not, choose a path to take
        else:
            # Check for redundant paths, and consider them to be visited
            non_visited_paths = intersection.get_possible_paths() - intersection.get_visited_paths()
            available_paths = [(available, get_distance_to_closest_intersection_in_front_of_pos(intersection_pos, available, rdata.intersections))
                        for available in non_visited_paths]
            # Only consider the distance information from the get_distance_... function
            available_paths = [(available, closest[0] if closest is not None else None) for available, closest in available_paths]
                
            for path, distance in available_paths:
                if distance == 2:
                    # Update pmap
                    map_position = walk_in_direction(intersection_pos, path, 1)
                    rdata.pmap.append(map_position)

                    # Update visited paths
                    other_intersection_pos = walk_in_direction(intersection_pos, path, 2)
                    other_intersection = rdata.intersections[other_intersection_pos]
                    other_intersection.add_visited_path(opposite_direction(path))
                    other_intersection.add_neighbour(intersection)
                        
                    intersection.add_visited_path(path)
                    intersection.add_neighbour(other_intersection)

                    non_visited_paths.remove(path)

            # If the robot already took all possible paths (at least once)
            if not non_visited_paths:

                # Use wavefront expansion to find the path to the closest intersection with non-visited paths
                self.set_route_to(
                    key=lambda n: isinstance(n, Intersection) and (n.get_possible_paths() - n.get_visited_paths()),
                    direction=direction,
                    start_node=intersection,
                    rdata=rdata
                )
                # It can be empty if the path leads nowhere
                if rdata.intersections_intentions:
                    next_intention = rdata.intersections_intentions.pop(0)
            
            # If there are non-visited paths to take
            else:

                suitable_paths = [(path, distance) for path, distance in available_paths if not distance or distance > 2]
                available, _ = min(suitable_paths, key=lambda t: t[1] if t[1] is not None else 10)
                
                next_intention = None
                if direction == available:
                    next_intention = MoveForward()
                else:
                    next_intention = Rotate(direction, available)

        # If no path was explicitly chosen, then take a random path from the available paths (to avoid loops)
        if not next_intention:
            available = random.choice(list(intersection.get_possible_paths()))
            self.acted_randomly = True

            if direction == available:
                return None, MoveForward()
            else:
                return None, Rotate(direction, available)
        
        rdata.previous_intersection = intersection

        return None, next_intention
            

class Rotate(Intention):

    def __init__(self, starting_direction: Direction, end_direction: Direction, advancement_steps: int=4):
        super().__init__()

        self.end_direction = end_direction
        self.left = left_direction(starting_direction) == end_direction
        self.advancement_steps = advancement_steps
        self.count = 0
    
    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        if self.count < self.advancement_steps:
            left_motor = right_motor = self.velocity
            self.count += 1
        else:
            left_motor = self.velocity if not self.left else -self.velocity
            right_motor = self.velocity if self.left else -self.velocity

        next_intention = None
        if get_direction(measures.compass) == self.end_direction and abs(get_angle_to_track(measures.compass)) < 30:

            next_intention = Wander()
        
        return (left_motor, right_motor), next_intention
    
    def __str__(self): return 'Rotate ' +  ('left' if self.left else 'right')
    def __repr__(self): return str(self)


class MoveForward(Intention):

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        left_motor = right_motor = self.velocity

        count = 0
        for ls in measures.lineSensor:
            if ls == '1':
                count += 1 
            elif ls == '0' and count > 0:
                break

        next_intention = None
        if count <= 3:
            next_intention = Wander()

        return (left_motor, right_motor), next_intention
    
    def __str__(self): return 'Move forward'
    def __repr__(self): return str(self)


class TurnBack(Intention):
    
    def __init__(self):
        super().__init__()

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        if len([ls for ls in measures.lineSensor if ls=='1']) > 2:
            return (0.0, 0.0), Wander()
        
        return (self.velocity, -self.velocity), TurnBack()


class SampleLoop(Intention):

    def __init__(self, next_intention: Intention, *args, **kwargs):
        self.next_intention = next_intention
        self.next_intention_args = args
        self.next_intention_kwargs = kwargs
        self.n_samples = 5   # should be odd to avoid ties
        self.n_samples_counter = 0

        self.lineSensor = []

    def most_common(self, l: Sequence[str]) -> str:
        return '0' if l.count('0') > self.n_samples / 2 else '1'

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        self.lineSensor.append(measures.lineSensor)
        self.n_samples_counter += 1
        
        if rdata.expected_noise and self.n_samples_counter < self.n_samples:
            return (0.0, 0.0), None
        
        self.lineSensor = [self.most_common(samples) for samples in zip(*self.lineSensor)]
        
        return (0.0, 0.0), self.next_intention(*self.next_intention_args, **self.next_intention_kwargs, sample_loop=self)


class GoToStartingPosition(Intention):

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        (x, y), position = self.obtain_position(measures, rdata)
        
        direction = get_direction(measures.compass)
        closest = get_walkable_distance_to_closest_intersection_in_front_of_pos((x, y), direction, rdata.intersections.keys(), rdata.pmap, position)
        # Should always happen! But if not so, just wander and try again later
        if closest is not None:
            intersection_in_front = rdata.intersections[closest[1]]

            # Calculate route to the starting position
            update_checkpoints_neighbours(rdata)
            self.set_route_to(
                key=lambda n: isinstance(n, Checkpoint) and n.get_coordinates() == rdata.starting_position,
                direction=get_direction(measures.compass),
                start_node=intersection_in_front,
                rdata=rdata,
            )

        return None, Wander()


class Finish(Intention):

    def __init__(self):
        super().__init__()

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        print('Number of noise discontinuities on lineSensors:', rdata.discontinuities)
        return (0.0, 0.0), None
