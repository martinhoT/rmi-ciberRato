import math
import random

from os import system
from typing import Dict, List, Tuple
from croblink import CMeasures
from intersection import Intersection
from robData import RobData

from directions import Direction, left_direction, opposite_direction, right_direction
from mapper import map_to_text



LOG_CLEAR = True
LOG_STARTING_POS = False
LOG_INTENTION = True
LOG_SENSORS = True
LOG_INTERSECTIONS = True
LOG_MAP = True
LOG_CALCULATED_PATH = True

SPEED_OPTIMIZATIONS = False

# TODO: Optimization for C2: When choosing new directions to take, choose the one that will lead with one step to a known intersection first, even if the path is unknown (might happen, see bottom-left corner of C2's maps)
class Intention:

    def __init__(self):
        self.velocity = 0.08

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        raise NotImplementedError()

    # Obtain orientation of robot in the map
    def getDirection(self, measures: CMeasures):
        if measures.compass >= 45 and measures.compass <= 135:
            return Direction.N
        elif measures.compass < 45 and measures.compass > -45:
            return Direction.E
        elif measures.compass <= -45 and measures.compass >= -135:
            return Direction.S
        else:
            return Direction.W
    
    def round_pos(self, x: float, y: float, starting_position: Tuple[float, float]):
        # Assumed that the robot is still in the starting position and hasn't updated it
        if not starting_position:
            return 0, 0
        # Works diffrently for negative numbers!
        # return math.floor(x-starting_position[0] + 0.5), math.floor(y-starting_position[1] + 0.5)
        return round(x-starting_position[0]), round(y-starting_position[1])

    def round_pos_to_intersection(self, x: float, y: float, starting_position: Tuple[float, float]):
        if not starting_position:
            return 0, 0
        return round((x-starting_position[0])/2)*2, round((y-starting_position[1])/2)*2

    def log_measured(self, measures: CMeasures, rdata: RobData):
        if LOG_CLEAR:
            system('clear')
        if LOG_STARTING_POS:
            print(rdata.starting_position)
        if LOG_INTENTION:
            print(self.__class__.__name__)
        if LOG_SENSORS:
            print(f'{measures.lineSensor} ({measures.x}, {measures.y}) -> ({self.round_pos(measures.x, measures.y, rdata.starting_position)})')
        if LOG_INTERSECTIONS:
            print('Intersections:')
            for position, intersection in rdata.intersections.items():
                not_visited = intersection.get_possible_paths() - intersection.get_visited_paths()
                if not_visited:
                    print(position, "- Not Visited:" , not_visited, "- Neighbours:", intersection.get_neighbours())
        if LOG_CALCULATED_PATH:
            print('Calculated Path:')
            print('Robot Position:', self.round_pos(measures.x, measures.y, rdata.starting_position))
            print(rdata.path)
            print(rdata.intersections_intentions)
        if LOG_MAP and rdata.pmap:
            for line in map_to_text(list(rdata.pmap)):
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
    # Only works with RobC2 since the compass is used
    def follow_path(self, measures: CMeasures) -> Tuple[int, int]:
        angle_to_track = self.get_angle_to_track(measures)

        left = measures.lineSensor[:3].count("1")
        right = measures.lineSensor[4:].count("1")

        left_imbalance = left - right

        return (self.velocity * (1 - (-angle_to_track/45) - (left*1/3 if left_imbalance > 0 else 0)), 
                self.velocity * (1 - (angle_to_track/45) - (right*1/3 if left_imbalance < 0 else 0)))

    def get_angle_to_track(self, measures: CMeasures):
        direction = self.getDirection(measures)
        return measures.compass - \
                 (90 if direction == Direction.N
            else -90 if direction == Direction.S
            else  180 if measures.compass > 135
            else -180 if measures.compass < -135
            else 0)
    
    @classmethod
    def calculate_moves(cls, direction: Direction, path: List[Intersection]) -> List['Intention']:
        
        moves = []
        for i in range(len(path) - 1):

            new_direction = cls.get_direction_from_path(path[i], path[i+1])

            if direction == new_direction:
                move = MoveForward()
            else:
                move = Rotate(direction, new_direction)

            moves.append(move)
            direction = new_direction

        return moves
        
    @classmethod
    def get_direction_from_path(cls, start: Intersection, end: Intersection) -> Direction:

        # Same x
        if start.get_x() == end.get_x():
            if end.get_y() > start.get_y():
                return Direction.N
            else:
                return Direction.S
        
        # Same y
        else:
            if end.get_x() > start.get_x():
                return Direction.E
            else:
                return Direction.W

    @classmethod
    def get_walkable_distance_to_closest_intersection_in_front_of_pos(cls,
            position: Tuple[int, int],
            direction: Direction,
            intersections: Dict[Intersection, Direction],
            pmap: List[Tuple[int, int]]):

        # If the intersection in front of me is far away, then speed up
        intersection_in_front_distance = {
            Direction.E: lambda i, p: i[0] - p[0] if i[1] == p[1] else -1,
            Direction.W: lambda i, p: p[0] - i[0] if i[1] == p[1] else -1,
            Direction.N: lambda i, p: i[1] - p[1] if i[0] == p[0] else -1,
            Direction.S: lambda i, p: p[1] - i[1] if i[0] == p[0] else -1
        }[direction]

        distance_of_intersections_in_front_of_me = [intersection_in_front_distance(intersection, position) for intersection in intersections
            if intersection_in_front_distance(intersection, position) > 0]

        if SPEED_OPTIMIZATIONS and distance_of_intersections_in_front_of_me:
            # There needs to be straight path to the intersection
            closest_distance = min(distance_of_intersections_in_front_of_me)
            
            intersection_step = {
                Direction.E: lambda i, n: (i[0] + n, i[1]),
                Direction.W: lambda i, n: (i[0] - n, i[1]),
                Direction.N: lambda i, n: (i[0], i[1] + n),
                Direction.S: lambda i, n: (i[0], i[1] - n)
            }[direction]

            positions_to_be_covered = {intersection_step(position, n) for n in range(1, closest_distance + 1)}
            
            # Should reach that intersection in a known straight path from this position
            if len(positions_to_be_covered - set(pmap)) == 0:
                return closest_distance
        
        return None

    def __str__(self): return self.__class__.__name__
    def __repr__(self): return str(self)


class Wander(Intention):

    def __init__(self):
        super().__init__()
        self.count = 0

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        if not rdata.starting_position:
            rdata.starting_position = (measures.x, measures.y)

        n_active = measures.lineSensor.count("1")

        # Obtain position of robot in the map
        position = self.round_pos(measures.x, measures.y, rdata.starting_position)
        if position not in rdata.pmap:
            rdata.pmap.append(position)
        
        direction = self.getDirection(measures)

        # Robot is off track
        if (n_active == 0):
            return (0.0, 0.0), TurnBack(direction)

        # Robot is on track
        # TODO: watch out for flipped bits, might ruin everything
        left = measures.lineSensor[:5].count("1")
        right = measures.lineSensor[3:].count("1")

        # leftTurn = left == 3
        # rightTurn = right == 3
        leftTurn = left >= 3 and measures.lineSensor[0] == "1"
        rightTurn = right >= 3 and measures.lineSensor[6] == "1"

        # Possible intersection found
        if (leftTurn or rightTurn) and measures.lineSensor[3] == '1':

            # Adjust position to the closest possible intersection
            position = self.round_pos_to_intersection(measures.x, measures.y, rdata.starting_position)

            # If the intersection is not in the map, add it
            if position not in rdata.intersections:

                # state.intersections[position] = Intersection()
                rdata.intersections[position] = Intersection(position[0], position[1])
                
                # Add neighbours
                if rdata.current_intersection:
                    rdata.intersections[position].add_neighbour(rdata.current_intersection)
                    rdata.intersections[rdata.current_intersection].add_neighbour(position)

                # Update current neighbour
                rdata.current_intersection = position

                # Add current path to intersection
                rdata.intersections[position].add_path(opposite_direction(direction))
                rdata.intersections[position].add_visited_path(opposite_direction(direction))

                if leftTurn:
                    rdata.intersections[position].add_path(left_direction(direction))

                if rightTurn:
                    rdata.intersections[position].add_path(right_direction(direction))
            
                return (0.0, 0.0), CheckIntersectionForward(position)

            # All known intersections have been exhausted, which should mean that the entire map has been traversed
            elif len(rdata.intersections) != 0 \
                and all(len(intersection.get_possible_paths() - intersection.get_visited_paths()) == 0 for intersection in rdata.intersections.values()):

                return (0.0, 0.0), Finish()

            # If the intersection is in the map
            else:

                # Add last intersection as neighbour
                if position != rdata.current_intersection:
                    rdata.intersections[position].add_neighbour(rdata.current_intersection)
                    rdata.intersections[rdata.current_intersection].add_neighbour(position)

                    # Update current neighbour
                    rdata.current_intersection = position

                # If there are pre-calculated intentions for this intersection, follow them
                intersection = rdata.intersections[position]
                if intersection in rdata.path:

                    # Inbetween intersections
                    if rdata.intersections_intentions:
                        rdata.path.remove(intersection)
                        return (0.0, 0.0), rdata.intersections_intentions.pop(0)
                    
                    #  Reached the end of the path
                    else:
                        rdata.path.pop()

                # Check if it is a new direction
                if opposite_direction(direction) not in rdata.intersections[position].get_visited_paths():
                    rdata.intersections[position].add_visited_path(opposite_direction(direction))

                # If the robot already took all possible paths (at least once)
                non_visited_paths = rdata.intersections[position].get_possible_paths() - rdata.intersections[position].get_visited_paths()
                if not non_visited_paths:

                    # Obtain path to closest intersection with non visited paths
                    path_to_closest_intersection = None

                    neighbours = rdata.intersections[position].get_neighbours()
                    intersections = [(rdata.intersections[position], [], 0)]
                    checked_intersections = []
                    previous_intersections = []
                    distance_to_this_point = lambda t: t[2]

                    # Wavefront expansion to find closest intersection and path to it
                    while neighbours and intersections and not path_to_closest_intersection:

                        intersections.sort(key=distance_to_this_point, reverse=True)
                        this_intersection, previous_intersections, previous_distance = intersections.pop()
                        neighbours = this_intersection.get_neighbours()

                        checked_intersections.append(this_intersection)

                        if neighbours:

                            for neighbour in neighbours:

                                neighbour_intersection = rdata.intersections[neighbour]
                                non_visited_paths = neighbour_intersection.get_possible_paths() - neighbour_intersection.get_visited_paths()
                                
                                distance_x = abs(this_intersection.get_x() - neighbour_intersection.get_x())
                                distance_y = abs(this_intersection.get_y() - neighbour_intersection.get_y())
                                distance = distance_x + distance_y
                                
                                if non_visited_paths:
                                    path_to_closest_intersection = previous_intersections + [this_intersection, neighbour_intersection]
                                    break

                                if neighbour_intersection not in checked_intersections:
                                    intersections.append((neighbour_intersection, previous_intersections + [this_intersection], previous_distance + distance))

                    if path_to_closest_intersection:
                        rdata.intersections_intentions = Intention.calculate_moves(direction, path_to_closest_intersection)
                        rdata.path = path_to_closest_intersection[1:]
                        return (0.0, 0.0), rdata.intersections_intentions.pop(0)
                        
                    else:
                        # Random path from available paths (to avoid loops)
                        available = random.choice(list(rdata.intersections[position].get_possible_paths()))

                        if direction == available:
                            return None, MoveForward()
                        else:
                            return None, Rotate(direction, available)

                else:
                    return (0.0, 0.0), TurnIntersection()
        
        extra_velocity = 0
        if SPEED_OPTIMIZATIONS:
            closest_distance = Intention.get_walkable_distance_to_closest_intersection_in_front_of_pos(position, direction, rdata.intersections, rdata.pmap)
            if closest_distance:
                x = closest_distance/4
                if x < 0.25:
                    extra_velocity = -2*x
                if x > 1:
                    extra_velocity = 1
                else:
                    extra_velocity = x
            
        action = self.follow_path(measures)
        return (action[0]*(1 + extra_velocity), action[1]*(1 + extra_velocity)), None


class CheckIntersectionForward(Intention):

    def __init__(self, intersection_pos: Tuple[int, int], test_steps=5):
        super().__init__()
        self.intersection_pos = intersection_pos
        self.test_steps = test_steps        

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        next_intention = None

        if all(ls == '0' for ls in measures.lineSensor):
            next_intention = CheckIntersectionForwardBacktrack(self.intersection_pos, False)

        if self.test_steps == 0:
            next_intention = CheckIntersectionForwardBacktrack(self.intersection_pos, True)

        self.test_steps -= 1

        return (self.velocity, self.velocity), next_intention


class CheckIntersectionForwardBacktrack(Intention):

    def __init__(self, intersection_pos: Tuple[int, int], has_intersection_forward: bool):
        super().__init__()
        self.intersection_pos = intersection_pos
        self.has_intersection_forward = has_intersection_forward

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        direction = self.getDirection(measures)

        # If the robot is back at the intersection
        if all(ls == '1' for ls in measures.lineSensor[:3]) or all(ls == '1' for ls in measures.lineSensor[4:]):
            
            if self.has_intersection_forward:

                intersection = self.round_pos_to_intersection(measures.x, measures.y, rdata.starting_position)
                rdata.intersections[intersection].add_path(direction)

            return (self.velocity, self.velocity), TurnIntersection()

        return (-self.velocity, -self.velocity), None


# TODO: extract to a simple function? It's only used for a single step
class TurnIntersection(Intention):

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        position = self.round_pos_to_intersection(measures.x, measures.y, rdata.starting_position)
        direction = self.getDirection(measures)

        non_visited_paths = rdata.intersections[position].get_possible_paths() - rdata.intersections[position].get_visited_paths()
        
        available = non_visited_paths.pop()
        rdata.intersections[position].add_visited_path(available)

        next_intention = None
        if left_direction(direction) == available:
            next_intention = Rotate(direction, left_direction(direction))
            
        elif right_direction(direction) == available:
            next_intention = Rotate(direction, right_direction(direction))

        else:
            next_intention = MoveForward()
        
        return None, next_intention
            

class Rotate(Intention):

    def __init__(self, starting_direction: Direction, end_direction: Direction=None, advancement_steps: int=4):
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
        if self.getDirection(measures) == self.end_direction and abs(self.get_angle_to_track(measures)) < 30:

            # Update visited path
            direction = self.getDirection(measures)
            intersection = self.round_pos_to_intersection(measures.x, measures.y, rdata.starting_position)
            rdata.intersections[intersection].add_visited_path(direction)

            next_intention = Wander()
        
        return (left_motor, right_motor), next_intention
    
    def __str__(self): return 'Rotate ' +  ('left' if self.left else 'right')
    def __repr__(self): return str(self)


class MoveForward(Intention):

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        self.log_measured(measures, rdata)

        # Update visited path
        direction = self.getDirection(measures)        
        intersection = self.round_pos_to_intersection(measures.x, measures.y, rdata.starting_position)
        
        rdata.intersections[intersection].add_visited_path(direction)

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

    def __init__(self, starting_direction: Direction):
        super().__init__()
        self.starting_direction = starting_direction
        self.count = 0

    def act(self, measures: CMeasures, rdata: RobData) -> Tuple[Tuple[float, float], 'Intention']:
        n_active = measures.lineSensor.count("1")

        next_intention = Wander() if n_active != 0 else None
        
        return (self.velocity, -self.velocity), next_intention
    
    def __str__(self): return 'Turn back'
    def __repr__(self): return str(self)


class Finish(Intention):

    def __init__(self):
        super().__init__()

    def act(self, measures: CMeasures, rdata: RobData):
        return (0.0, 0.0), None