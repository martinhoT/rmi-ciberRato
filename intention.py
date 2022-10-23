from ast import Pass
import math

from os import system
from typing import Tuple, Union, TYPE_CHECKING
from croblink import CMeasures
from intersection import intersection
from robstate import RobState

from directions import Direction, left_direction, opposite_direction, right_direction
from mapper import map_to_text



LOG_CLEAR = True
LOG_STARTING_POS = True
LOG_INTENTION = True
LOG_SENSORS = True
LOG_INTERSECTIONS = True
LOG_MAP = True

SPEED_OPTIMIZATIONS = True

class Intention:

    def __init__(self):
        self.velocity = 0.08
        # print(self.__class__.__name__, 'instanced')

    def act(self, measures: CMeasures, state: RobState) -> Tuple[float, float, 'Intention']:
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

    def log_measured(self, measures: CMeasures, state: RobState):
        if LOG_CLEAR:
            system('clear')
        if LOG_STARTING_POS:
            print(state.starting_position)
        if LOG_INTENTION:
            print(self.__class__.__name__)
        if LOG_SENSORS:
            print(f'{measures.lineSensor} ({measures.x}, {measures.y}) -> ({self.round_pos(measures.x, measures.y, state['starting_position'])})')
        if LOG_INTERSECTIONS:
            print('Intersections:')
            for position, intersection in state.intersections.items():
                not_visited = intersection.get_possible_paths() - intersection.get_visited_paths()
                # if not_visited:
                    # print(position, "- Not Visited:" , not_visited, "- Neighbours:", intersection.get_neighbours())
                # print(position, "- Possible:" , intersection.get_possible_paths(), "- Visited:", intersection.get_visited_paths())
                print(position, "- Not Visited:" , not_visited, "- Neighbours:", intersection.get_neighbours())
            """
            for intersection, (possible_directions, non_visited_directions) in intersections.items():
                if non_visited_directions:
                    print(intersection, non_visited_directions)
            """
        if LOG_MAP and state.map:
            for line in map_to_text(list(state.map)):
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

        return (self.velocity, self.velocity)
        # return (0.15, 0.15) # Max speed

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


class Wander(Intention):

    def act(self, measures: CMeasures, state: RobState) -> Tuple[float, float, 'Intention']:
        self.log_measured(measures, state)

        if not state.starting_position:
            state.starting_position = (measures.x, measures.y)

        n_active = measures.lineSensor.count("1")

        # Obtain position of robot in the map
        position = self.round_pos(measures.x, measures.y, state.starting_position)
        if position not in state.map:
            state.map.append(position)

        # Robot is off track
        if (n_active == 0):
            return 0.0, 0.0, TurnBack( self.getDirection(measures) )

        # Robot is on track
        left = measures.lineSensor[:3].count("1")
        right = measures.lineSensor[4:].count("1")

        leftTurn = left == 3
        rightTurn = right == 3
        
        direction = self.getDirection(measures)

        # Possible intersection found
        if (leftTurn or rightTurn) and measures.lineSensor[3] == '1':

            # Adjust position to the closest possible intersection
            position = self.round_pos_to_intersection(measures.x, measures.y, state.starting_position)

            # If the intersection is not in the map, add it
            if position not in state.intersections:

                # state.intersections[position] = Intersection()
                state.intersections[position] = Intersection(position[0], position[1])
                
                # Add neighbours
                if state.current_intersection:
                    state.intersections[position].add_neighbour(state.current_intersection)
                    state.intersections[state.current_intersection].add_neighbour(position)

                # Update current neighbour
                state.current_intersection = position

                # Add current path to intersection
                state.intersections[position].add_path(opposite_direction(direction))
                state.intersections[position].add_visited_path(opposite_direction(direction))

                if leftTurn:
                    print('NEW INTERSECTION at', position, 'in direction', left_direction(direction))
                    state.intersections[position].add_path(left_direction(direction))

                if rightTurn:
                    print('NEW INTERSECTION at', position, 'in direction', right_direction(direction))
                    state.intersections[position].add_path(right_direction(direction))
            
                return 0.0, 0.0, CheckIntersectionForward(position)

            # If the intersection is in the map, check if it is a new direction
            else:

                if opposite_direction(direction) not in state.intersections[position].get_visited_paths():
                    state.intersections[position].add_visited_path(opposite_direction(direction))

                # If the robot already took all possible paths (at least once)
                non_visited_paths = state.intersections[position].get_possible_paths() - state.intersections[position].get_visited_paths()
                if not non_visited_paths:

                    # Obtain closest intersection with non visited paths
                    closest_intersection = None

                    neighbours = state.intersections[position].get_neighbours()
                    intersections = [state.intersections[position]]
                    checked_intersections = []

                    # Wavefront expansion to find closest intersection and path to it
                    while neighbours and intersections and not closest_intersection:

                        # TODO: take into account the distance between neighbours? As in, only pop the closest one?
                        this_intersection = intersections.pop()
                        neighbours = this_intersection.get_neighbours()

                        checked_intersections.append(this_intersection)

                        if neighbours:

                            for neighbour in neighbours:

                                neighbour_intersection = intersections[neighbour]
                                non_visited_paths = neighbour_intersection.get_possible_paths() - neighbour_intersection.get_visited_paths()
                                if non_visited_paths:
                                    closest_intersection = neighbour_intersection
                                    break

                                if neighbour_intersection not in checked_intersections:
                                    intersections.append(neighbour_intersection)

                    # TODO: do we need to recalculate the path again? If we found the destination in the previous loop, then we already know the path of neighbors there
                    if closest_intersection:
                        return 0.0, 0.0, CalculatePath(position, neighbour)
                    if leftTurn:
                        return 0.0, 0.0, Rotate(True, self.getDirection(measures))
                    elif rightTurn:
                        return 0.0, 0.0, Rotate(False, self.getDirection(measures))

                else:
                    return 0.0, 0.0, TurnIntersection()
        
        # If the intersection in front of me is far away, then speed up
        intersection_in_front_distance = {
            Direction.E: lambda i, p: i[0] - p[0] if i[1] == p[1] else -1,
            Direction.W: lambda i, p: p[0] - i[0] if i[1] == p[1] else -1,
            Direction.N: lambda i, p: i[1] - p[1] if i[0] == p[0] else -1,
            Direction.S: lambda i, p: p[1] - i[1] if i[0] == p[0] else -1
        }[direction]

        distance_of_intersections_in_front_of_me = [intersection_in_front_distance(intersection, position) for intersection in state.intersections
            if intersection_in_front_distance(intersection, position) > 0]

        extra_velocity = 0
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
            if len(positions_to_be_covered - set(state.map)) == 0:
                x = closest_distance/4
                if x < 0.25:
                    extra_velocity = -2*x
                if x > 1:
                    extra_velocity = 1
                else:
                    extra_velocity = x
            
        action = self.follow_path(measures)
        return action[0]*(1 + extra_velocity), action[1]*(1 + extra_velocity), None


class CheckIntersectionForward(Intention):

    def __init__(self, intersection_pos: Tuple[int, int], test_steps=5):
        super().__init__()
        self.intersection_pos = intersection_pos
        self.test_steps = test_steps        

    def act(self, measures: CMeasures, state: RobState) -> Tuple[float, float, 'Intention']:
        self.log_measured(measures, state)

        next_intention = None

        if all(ls == '0' for ls in measures.lineSensor):
            next_intention = CheckIntersectionForwardBacktrack(self.intersection_pos, False)

        if self.test_steps == 0:
            next_intention = CheckIntersectionForwardBacktrack(self.intersection_pos, True)

        self.test_steps -= 1

        return self.velocity, self.velocity, next_intention


class CheckIntersectionForwardBacktrack(Intention):

    def __init__(self, intersection_pos: Tuple[int, int], has_intersection_forward: bool):
        super().__init__()
        self.intersection_pos = intersection_pos
        self.has_intersection_forward = has_intersection_forward

    def act(self, measures: CMeasures, state: RobState) -> Tuple[float, float, 'Intention']:
        self.log_measured(measures, state)

        direction = self.getDirection(measures)

        # If the robot is back at the intersection
        if all(ls == '1' for ls in measures.lineSensor[:3]) or all(ls == '1' for ls in measures.lineSensor[4:]):
            
            if self.has_intersection_forward:

                current_intersection = state.current_intersection
                state.intersections[current_intersection].add_path(direction)

            return self.velocity, self.velocity, TurnIntersection()

        return -self.velocity, -self.velocity, None


class TurnIntersection(Intention):

    def act(self, measures: CMeasures, state: RobState) -> Tuple[float, float, 'Intention']:
        self.log_measured(measures, state)

        position = self.round_pos_to_intersection(measures.x, measures.y, state.starting_position)
        direction = self.getDirection(measures)

        print('About to turn, possible directions:', state.intersections[position].get_possible_paths())
        non_visited_paths = state.intersections[position].get_possible_paths() - state.intersections[position].get_visited_paths()
        
        available = non_visited_paths.pop()
        state.intersections[position].add_visited_path(available)
        print("Taking direction", available)

        next_intention = {
            left_direction(direction): Rotate(True, direction),
            right_direction(direction): Rotate(False, direction),
            direction: MoveForward(),
        }[available]
        
        return 0.0, 0.0, next_intention
            

class Rotate(Intention):

    def __init__(self, left: bool, starting_direction: Direction):
        super().__init__()
        self.left = left
        self.starting_direction = starting_direction
        self.count = 0
    
    def act(self, measures: CMeasures, state: RobState) -> Tuple[float, float, 'Intention']:
        self.log_measured(measures, state)

        if self.count < 4:
            left_motor = right_motor = self.velocity
            self.count += 1
        else:
            left_motor = self.velocity if not self.left else -self.velocity
            right_motor = self.velocity if self.left else -self.velocity

        next_intention = None
        if self.getDirection(measures) != self.starting_direction and abs(self.get_angle_to_track(measures)) < 30:

            # Update visited path
            direction = self.getDirection(measures)        
            current_intersection = state.current_intersection
            state.intersections[current_intersection].add_visited_path(direction)

            next_intention = Wander()
        
        return left_motor, right_motor, next_intention


class MoveForward(Intention):

    def act(self, measures: CMeasures, state: RobState) -> Tuple[float, float, 'Intention']:
        self.log_measured(measures, state)

        # Update visited path
        direction = self.getDirection(measures)        
        current_intersection = state.current_intersection
        state.intersections[current_intersection].add_visited_path(direction)

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

        return left_motor, right_motor, next_intention


class TurnBack(Intention):

    def __init__(self, starting_direction: Direction):
        super().__init__()
        self.starting_direction = starting_direction
        self.count = 0

    def act(self, measures: CMeasures, state: RobState) -> Tuple[float, float, 'Intention']:
        self.log_measured(measures, state)

        if self.count < 3:
            left_motor, right_motor = -self.velocity, -self.velocity
            self.count += 1
        else:
            left_motor, right_motor = self.velocity, -self.velocity

        next_intention = None
        if self.getDirection(measures) == opposite_direction(self.starting_direction) and abs(self.get_angle_to_track(measures)) < 35:
            next_intention = Wander()
        
        return left_motor, right_motor, next_intention


class CalculatePath(Intention):

    def __init__(self, start: Tuple[int, int], end: Tuple[int, int]):
        super().__init__()
        self.start = start
        self.end = end
        self.path = None
        self.moves = None
        self.current_move = None

    def act(self, measures: CMeasures, state: RobState) -> Tuple[float, float, 'Intention']:
        self.log_measured(measures, state)

        # First time calculating the path
        if self.path is None:
            self.path = self.find_path(state.intersections, self.start, self.end)

        # First time calculating moves
        if self.moves is None:
            direction = self.getDirection(measures) 

            self.moves = []
            self.calculate_moves(direction)

        # print("Path:", self.path)
        # print("Moves:", self.moves)

        left_motor = right_motor = 0.0
        next_intention = None
        if self.path:
            
            position = self.round_pos_to_intersection(measures.x, measures.y, state.starting_position)
                
            # If the robot found an intersection
            if position in self.path and position != self.end:
                # Remove the intersection from the path
                self.path.remove(position)
                # Obtain the next move
                self.current_move = self.moves.pop(0)

            if self.current_move:
                # TODO: Handle current move
                pass

            else:
                # Drive forward until next intersection
                left_motor = right_motor = self.velocity

            # If the robot reached the end
            if position == self.end:
                next_intention = Wander()
        
        return left_motor, right_motor, next_intention

    def find_path(self, intersections, start, end, path=[]):
        
        path = path + [start]
        if start == end:
            return path
        
        if not start in intersections:
            return None

        for neighbour in intersections[start].get_neighbours():
            if neighbour not in path:
                new_path = self.find_path(intersections, neighbour, end, path)
                if new_path: 
                    return new_path

        return None

    def calculate_moves(self, direction):
        
        for i in range(len(self.path) - 1):

            new_direction = self.get_direction_from_path(self.path[i], self.path[i+1])

            print(direction, new_direction)

            if direction == new_direction:
                # self.moves.append('Foward')
                # move = (self.velocity, self.velocity)
                move = MoveForward()

            # TODO: is this correct, or switched up?
            if (direction.value - 1) % 4 == new_direction.value:
                # self.moves.append("Right")
                # move = (self.velocity, -self.velocity)
                move = Rotate(False, direction)
            
            elif (direction.value + 1) % 4 == new_direction.value:
                # self.moves.append("Left")
                # move = (-self.velocity, self.velocity)
                move = Rotate(True, direction)

            else: 
                # self.moves.append("Back")
                # move = (-self.velocity, -self.velocity)
                move = TurnBack(direction)

            direction = new_direction
        
    def get_direction_from_path(self, start, end):

        # Same x
        if start[0] == end[0]:
            if end[1] > start[1]:
                return Direction.N
            else:
                return Direction.S
        
        # Same y
        else:
            if end[0] > start[0]:
                return Direction.W
            else:
                return Direction.E