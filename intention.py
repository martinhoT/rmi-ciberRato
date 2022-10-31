import random

from os import system
from typing import Dict, List, Set, Tuple
from croblink import CMeasures
from graph import Checkpoint, Intersection, Node
from robData import RobData

from directions import Direction, left_direction, opposite_direction, right_direction
from utils import *


LOG = True
LOG_CLEAR = False
LOG_STARTING_POS = False
LOG_INTENTION = True
LOG_SENSORS = True
LOG_INTERSECTIONS = False
LOG_CALCULATED_PATH = False
LOG_GROUND = False
LOG_CHECKPOINTS = False
LOG_DISTANCE_KNOWN_INTERSECTION_AHEAD = False
LOG_MAP = False

SPEED_OPTIMIZATIONS = True

class Intention:

    def __init__(self):
        self.velocity = 0.08

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        raise NotImplementedError()

    def log_measured(self, measures: CMeasures, rdata: RobData):
        if LOG:
            if LOG_CLEAR:
                system('clear')
            if LOG_STARTING_POS:
                print(rdata.starting_position)
            if LOG_INTENTION:
                print(self)
            if LOG_SENSORS:
                print(f'{measures.lineSensor} ({measures.x}, {measures.y}) -> ({round_pos(measures.x, measures.y, rdata.starting_position)})')
            if LOG_INTERSECTIONS:
                print('Intersections:')
                for position, intersection in rdata.intersections.items():
                    not_visited = intersection.get_possible_paths() - intersection.get_visited_paths()
                    if not_visited:
                        print(position, "- Not Visited:" , not_visited, "- Neighbours:", intersection.get_neighbours())
                print('Previous intersection:', rdata.previous_intersection)
            if LOG_CALCULATED_PATH:
                print('Calculated Path:')
                print('Robot Position:', round_pos(measures.x, measures.y, rdata.starting_position))
                print('Next intersections:', rdata.path)
                print('Next intentions:', rdata.intersections_intentions)
            if LOG_GROUND:
                print('Ground:', measures.ground)
            if LOG_CHECKPOINTS:
                print('Checkpoints:', rdata.checkpoints)
            if LOG_DISTANCE_KNOWN_INTERSECTION_AHEAD:
                print('Distance to known intersection ahead:', get_walkable_distance_to_closest_intersection_in_front_of_pos(
                    (measures.x - rdata.starting_position[0], measures.y - rdata.starting_position[1]),
                    get_direction(measures.compass),
                    rdata.intersections, rdata.pmap,
                    round_pos(measures.x, measures.y, rdata.starting_position)))
            if LOG_MAP and rdata.pmap:
                for line in map_to_text(rdata.pmap):
                    print(''.join(line))

    def safeguard(self, measures: CMeasures):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        if measures.irSensor[center_id] > 5.0 \
           or measures.irSensor[left_id]  > 5.0 \
           or measures.irSensor[right_id] > 5.0 \
           or measures.irSensor[back_id]  > 5.0:
            return (-0.1, +0.1)
        elif measures.irSensor[left_id]> 2.7:
            return (0.1, 0.0)
        elif measures.irSensor[right_id]> 2.7:
            return (0.0, 0.1)

        # return (0.15, 0.15) # Max speed
        return (self.velocity, self.velocity)

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
    
    def speed_up_func(self, x, max_speed, velocity, slow_down_portion, speed_up_portion) -> float:
        a, b = slow_down_portion
        c, d = speed_up_portion
        
        if x < b:
            value = (max_speed - velocity)/velocity
            local_x = (x - a)/(b - a)

            if x < b - (b - a)/2:
                return 1 - value * 2**(20 * local_x - 11)
            return 1 - value * (1-2**(-20 * local_x + 9))

        elif x > c:
            value = (max_speed - velocity)/velocity
            local_x = (x - c)/(d - c)

            if x < d - (d - c)/2:
                return (max_speed/velocity) + value * 2**(20 * local_x - 11)
            return (max_speed/velocity) + value * (1-2**(-20 * local_x + 9))

        return max_speed/velocity

    def __str__(self): return self.__class__.__name__
    def __repr__(self): return str(self)


class Wander(Intention):

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        # Obtain position of robot in the map
        position = round_pos(measures.x, measures.y, rdata.starting_position)
        if position not in rdata.pmap:
            rdata.pmap.append(position)

        direction = get_direction(measures.compass)

        # Robot is off track
        n_active = measures.lineSensor.count("1")
        if (n_active == 0):
            return (0.0, 0.0), TurnBack()

        # Line Sensors detcted a discontinuity
        if self.line_sensor_discontinuity(measures.lineSensor):
            rdata.discontinuities += 1
            return (0.0, 0.0), None

        # Save checkpoint if one was found
        if (measures.ground != -1) and measures.ground not in rdata.checkpoints:
            checkpoint_pos = round_pos_to_intersection(measures.x, measures.y, rdata.starting_position)
            checkpoint = Checkpoint(checkpoint_pos[0], checkpoint_pos[1], measures.ground)
            rdata.checkpoints[measures.ground] = checkpoint
            
        # Possible intersection found
        if self.check_if_intersection(measures.lineSensor):

            # Adjust position to the closest possible intersection
            intersection_pos = round_pos_to_intersection(measures.x, measures.y, rdata.starting_position)

            # If the intersection is not in the map, add it
            if intersection_pos not in rdata.intersections:
                next_intention = self.create_intersection(intersection_pos, rdata.intersections)
                return (0.0, 0.0), next_intention
            
            # When the robot data suggests that the challenge has been finished
            if rdata.finished():
                return (0.0, 0.0), Finish()

            return None, TurnIntersection()
            
        velocity_modifier = 1
        if SPEED_OPTIMIZATIONS:
            closest_distance = get_walkable_distance_to_closest_intersection_in_front_of_pos(
                position=(measures.x - rdata.starting_position[0], measures.y - rdata.starting_position[1]),
                direction=direction, intersections=rdata.intersections, pmap=rdata.pmap, rounded_position=position)

            if closest_distance:
                max_speed = 0.15
                slow_down_portion = (0.8, 1.0)
                speed_up_portion = (1.5, 2.0)

                velocity_modifier = self.speed_up_func(closest_distance, max_speed, self.velocity, slow_down_portion, speed_up_portion)

        action = self.follow_path(measures)
        return (action[0]*velocity_modifier, action[1]*velocity_modifier), None
    
    def create_intersection(self, intersection_pos: Tuple[int, int],
            intersection_list: Dict[Tuple[int, int], Intersection]) -> Intention:
        
        intersection = Intersection(intersection_pos[0], intersection_pos[1])
        intersection_list[intersection_pos] = intersection

        return CheckIntersectionForward(intersection_pos)

    def check_if_intersection(self, lineSensor: List[str]) -> bool:

        # Robot is on track
        left = lineSensor[:4].count("1")
        right = lineSensor[3:].count("1")

        leftTurn = left >= 3 and lineSensor[0] == "1"
        rightTurn = right >= 3 and lineSensor[-1] == "1"

        return (leftTurn or rightTurn) and lineSensor[3] == '1'
        

class CheckIntersectionForward(Intention):

    def __init__(self, intersection_pos: Tuple[int, int], test_steps: int=5):
        super().__init__()
        self.intersection_pos = intersection_pos
        self.test_steps = test_steps
        self.found_directions = set()

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        direction = get_direction(measures.compass)
        next_intention = None

        if all(ls == '1' for ls in measures.lineSensor[:2]):
            self.found_directions.add(left_direction(direction))

        if all(ls == '1' for ls in measures.lineSensor[5:]):
            self.found_directions.add(right_direction(direction))

        if all(ls == '0' for ls in measures.lineSensor):
            next_intention = CheckIntersectionForwardBacktrack(self.intersection_pos, self.found_directions)

        if self.test_steps == 0:
            self.found_directions.add(direction)
            next_intention = CheckIntersectionForwardBacktrack(self.intersection_pos, self.found_directions)

        self.test_steps -= 1

        return (self.velocity, self.velocity), next_intention


class CheckIntersectionForwardBacktrack(Intention):

    def __init__(self, intersection_pos: Tuple[int, int], found_directions: Set[Direction]):
        super().__init__()
        self.intersection_pos = intersection_pos
        self.found_directions = found_directions

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        # If the robot is back at the intersection
        if all(ls == '1' for ls in measures.lineSensor[:3]) or all(ls == '1' for ls in measures.lineSensor[4:]):
            
            for found_direction in self.found_directions:

                intersection = round_pos_to_intersection(measures.x, measures.y, rdata.starting_position)
                rdata.intersections[intersection].add_path( found_direction )
            rdata.intersections[intersection].add_path( opposite_direction(get_direction(measures.compass)) )

            return (self.velocity, self.velocity), TurnIntersection()

        return (-self.velocity, -self.velocity), None


class TurnIntersection(Intention):

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        intersection_pos = round_pos_to_intersection(measures.x, measures.y, rdata.starting_position)
        direction = get_direction(measures.compass)
        intersection = rdata.intersections[intersection_pos]

        if rdata.previous_intersection:
            intersection.add_visited_path(opposite_direction(direction))
            # It's possible if finding dead-ends
            if rdata.previous_intersection != intersection:
                rdata.previous_intersection.add_visited_path(direction)

        self.update_neighbours(intersection, rdata)

        next_intention = None

        # If there are pre-calculated intentions, follow them
        if rdata.intersections_intentions:
            rdata.path.pop(0)
            next_intention = rdata.intersections_intentions.pop(0)
        
        # If not, choose a path to take
        else:
            # If the robot already took all possible paths (at least once)
            non_visited_paths = intersection.get_possible_paths() - intersection.get_visited_paths()
            if not non_visited_paths:

                # Wavefront expansion to find the path to the closest intersection with non-visited paths
                path_to_closest_intersection = wavefront_expansion(
                    start_node=intersection,
                    key=lambda n: isinstance(n, Intersection) and (n.get_possible_paths() - n.get_visited_paths()))

                if path_to_closest_intersection:
                    path, _ = path_to_closest_intersection
                    rdata.intersections_intentions = self.calculate_moves(direction, path)
                    rdata.path = path[1:]
                    next_intention = rdata.intersections_intentions.pop(0)
            
            # If there are non-visited paths to take
            else:
                available_paths = [(available, get_distance_to_closest_intersection_in_front_of_pos(intersection_pos, available, rdata.intersections))
                        for available in non_visited_paths]
                available, _ = min(available_paths, key=lambda t: t[1] if t[1] is not None else 10)
                
                next_intention = None
                if direction == available:
                    next_intention = MoveForward()
                else:
                    next_intention = Rotate(direction, available)

                # Add chosen path
                intersection.add_visited_path(available)

        # If no path was explicitly chosen, then take a random path from the available paths (to avoid loops)
        if not next_intention:
            available = random.choice(list(intersection.get_possible_paths()))

            if direction == available:
                return None, MoveForward()
            else:
                return None, Rotate(direction, available)
        
        rdata.previous_intersection = intersection

        return None, next_intention
            

class Rotate(Intention):

    def __init__(self, starting_direction: Direction, end_direction: Direction, advancement_steps: int=3):
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

        # Update visited path
        # direction = get_direction(measures.compass)
        # intersection = round_pos_to_intersection(measures.x, measures.y, rdata.starting_position)
        
        # rdata.intersections[intersection].add_visited_path(direction)

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

    def act(self, measures: CMeasures, rdata: RobData):
        
        if any(ls=='1' for ls in measures.lineSensor):
            return (0.0, 0.0), Wander()
        
        return (self.velocity, -self.velocity), TurnBack()


class Finish(Intention):

    def __init__(self):
        super().__init__()

    def act(self, measures: CMeasures, rdata: RobData):
        print('Number of noise discontinuities on lineSensors:', rdata.discontinuities)
        return (0.0, 0.0), None