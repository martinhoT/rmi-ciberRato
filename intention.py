from ast import Pass
import math

from os import system
from typing import Tuple, Union, TYPE_CHECKING
from croblink import CMeasures
from intersection import Intersection
if TYPE_CHECKING:
    from robC1 import MyRob as RobC1
    from robC2 import MyRob as RobC2

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

    def act(self, robot: Union['RobC1', 'RobC2']):
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
    
    def round_pos(self, x: float, y: float, robot: Union['RobC1', 'RobC2']):
        # Assumed that the robot is still in the starting position and hasn't updated it
        if not robot.starting_position:
            return 0, 0
        # Works diffrently for negative numbers!
        # return math.floor(x-robot.starting_position[0] + 0.5), math.floor(y-robot.starting_position[1] + 0.5)
        return round(x-robot.starting_position[0]), round(y-robot.starting_position[1])

    def round_pos_to_intersection(self, x: float, y: float, robot: Union['RobC1', 'RobC2']):
        if not robot.starting_position:
            return 0, 0
        return round((x-robot.starting_position[0])/2)*2, round((y-robot.starting_position[1])/2)*2

    def log_measured(self, robot: Union['RobC1', 'RobC2']):
        if LOG_CLEAR:
            system('clear')
        if LOG_STARTING_POS:
            print(robot.starting_position)
        if LOG_INTENTION:
            print(self.__class__.__name__)
        if LOG_SENSORS:
            print(f'{robot.measures.lineSensor} ({robot.measures.x}, {robot.measures.y}) -> ({self.round_pos(robot.measures.x, robot.measures.y, robot)})')
        if LOG_INTERSECTIONS:
            print('Intersections:')
            for position, intersection in robot.intersections.items():
                not_visited = intersection.get_possible_paths() - intersection.get_visited_paths()
                # if not_visited:
                    # print(position, "- Not Visited:" , not_visited, "- Neighbours:", intersection.get_neighbours())
                # print(position, "- Possible:" , intersection.get_possible_paths(), "- Visited:", intersection.get_visited_paths())
                print(position, "- Not Visited:" , not_visited, "- Neighbours:", intersection.get_neighbours())
            """
            for intersection, (possible_directions, non_visited_directions) in robot.intersections.items():
                if non_visited_directions:
                    print(intersection, non_visited_directions)
            """
        if LOG_MAP and robot.map:
            for line in map_to_text(list(robot.map)):
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

    def act(self, robot: 'RobC2'):
        self.log_measured(robot)

        # robot.history = 0 -> Straight
        # robot.history = 1 -> Left
        # robot.history = 2 -> Right

        if not robot.starting_position:
            robot.starting_position = (robot.measures.x, robot.measures.y)

        n_active = robot.measures.lineSensor.count("1")

        # Obtain position of robot in the map
        position = self.round_pos(robot.measures.x, robot.measures.y, robot)
        if position not in robot.map:
            robot.map.append(position)

        # Robot is off track
        if (n_active == 0):
            robot.driveMotors(0.0, 0.0)
            robot.intention = TurnBack( self.getDirection(robot.measures) )
            return

        # Robot is on track
        left = robot.measures.lineSensor[:3].count("1")
        right = robot.measures.lineSensor[4:].count("1")

        leftTurn = left == 3
        rightTurn = right == 3
        
        direction = self.getDirection(robot.measures)

        # Possible intersection found
        if (leftTurn or rightTurn) and robot.measures.lineSensor[3] == '1':

            # Adjust position to the closest possible intersection
            position = self.round_pos_to_intersection(robot.measures.x, robot.measures.y, robot)

            # If the intersection is not in the map, add it
            if position not in robot.intersections:

                # robot.intersections[position] = Intersection()
                robot.intersections[position] = Intersection(position[0], position[1])
                
                # Add neighbours
                if robot.current_intersection:
                    robot.intersections[position].add_neighbour(robot.current_intersection)
                    robot.intersections[robot.current_intersection].add_neighbour(position)

                # Update current neighbour
                robot.current_intersection = position

                # Add current path to intersection
                robot.intersections[position].add_path(opposite_direction(direction))
                robot.intersections[position].add_visited_path(opposite_direction(direction))

                if leftTurn:
                    print('NEW INTERSECTION at', position, 'in direction', left_direction(direction))
                    robot.intersections[position].add_path(left_direction(direction))

                if rightTurn:
                    print('NEW INTERSECTION at', position, 'in direction', right_direction(direction))
                    robot.intersections[position].add_path(right_direction(direction))
            
                robot.intention = CheckIntersectionForward(position)
                robot.driveMotors(0.0, 0.0)
                return

            # If the intersection is in the map, check if it is a new direction
            else:

                if opposite_direction(direction) not in robot.intersections[position].get_visited_paths():
                    robot.intersections[position].add_visited_path(opposite_direction(direction))

                # If the robot already took all possible paths (at least once)
                non_visited_paths = robot.intersections[position].get_possible_paths() - robot.intersections[position].get_visited_paths()
                if not non_visited_paths:

                    # Obtain closest intersection with non visited paths
                    closest_intersection = None

                    neighbours = robot.intersections[position].get_neighbours()
                    intersections = [robot.intersections[position]]
                    checked_intersections = []

                    # Wavefront expansion to find closest intersection and path to it
                    while neighbours and intersections and not closest_intersection:

                        # TODO: take into account the distance between neighbours? As in, only pop the closest one?
                        this_intersection = intersections.pop()
                        neighbours = this_intersection.get_neighbours()

                        checked_intersections.append(this_intersection)

                        if neighbours:

                            for neighbour in neighbours:

                                neighbour_intersection = robot.intersections[neighbour]
                                non_visited_paths = neighbour_intersection.get_possible_paths() - neighbour_intersection.get_visited_paths()
                                if non_visited_paths:
                                    closest_intersection = neighbour_intersection
                                    break

                                if neighbour_intersection not in checked_intersections:
                                    intersections.append(neighbour_intersection)

                    # TODO: do we need to recalculate the path again? If we found the destination in the previous loop, then we already know the path of neighbors there
                    if closest_intersection:
                        robot.intention = CalculatePath(position, neighbour)
                        return
                    if leftTurn:
                        robot.intention = Rotate(True, self.getDirection(robot.measures))
                    elif rightTurn:
                        robot.intention = Rotate(False, self.getDirection(robot.measures))

                else:
                    robot.intention = TurnIntersection()
        
        # If the intersection in front of me is far away, then speed up
        intersection_in_front_distance = {
            Direction.E: lambda i, p: i[0] - p[0] if i[1] == p[1] else -1,
            Direction.W: lambda i, p: p[0] - i[0] if i[1] == p[1] else -1,
            Direction.N: lambda i, p: i[1] - p[1] if i[0] == p[0] else -1,
            Direction.S: lambda i, p: p[1] - i[1] if i[0] == p[0] else -1
        }[direction]

        distance_of_intersections_in_front_of_me = [intersection_in_front_distance(intersection, position) for intersection in robot.intersections
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
            if len(positions_to_be_covered - set(robot.map)) == 0:
                x = closest_distance/4
                if x < 0.25:
                    extra_velocity = -2*x
                if x > 1:
                    extra_velocity = 1
                else:
                    extra_velocity = x
            
        action = self.follow_path(robot.measures)
        robot.driveMotors(action[0]*(1 + extra_velocity), action[1]*(1 + extra_velocity))


class CheckIntersectionForward(Intention):

    def __init__(self, intersection_pos: Tuple[int, int], test_steps=5):
        super().__init__()
        self.intersection_pos = intersection_pos
        self.test_steps = test_steps        

    def act(self, robot: 'RobC2'):
        self.log_measured(robot)

        robot.driveMotors(self.velocity, self.velocity)
        
        if all(ls == '0' for ls in robot.measures.lineSensor):
            robot.intention = CheckIntersectionForwardBacktrack(self.intersection_pos, False)

        if self.test_steps == 0:
            robot.intention = CheckIntersectionForwardBacktrack(self.intersection_pos, True)
        self.test_steps -= 1


class CheckIntersectionForwardBacktrack(Intention):

    def __init__(self, intersection_pos: Tuple[int, int], has_intersection_forward: bool):
        super().__init__()
        self.intersection_pos = intersection_pos
        self.has_intersection_forward = has_intersection_forward

    def act(self, robot: 'RobC2'):
        self.log_measured(robot)

        direction = self.getDirection(robot.measures)

        # If the robot is back at the intersection
        if all(ls == '1' for ls in robot.measures.lineSensor[:3]) or all(ls == '1' for ls in robot.measures.lineSensor[4:]):
            
            if self.has_intersection_forward:

                current_intersection = robot.current_intersection
                robot.intersections[current_intersection].add_path(direction)

            robot.driveMotors(self.velocity, self.velocity)
            robot.intention = TurnIntersection()

        else:
            robot.driveMotors(-self.velocity, -self.velocity)


class TurnIntersection(Intention):

    def act(self, robot: 'RobC2'):
        
        position = self.round_pos_to_intersection(robot.measures.x, robot.measures.y, robot)
        direction = self.getDirection(robot.measures)

        robot.driveMotors(0.0, 0.0)

        print('About to turn, possible directions:', robot.intersections[position].get_possible_paths())
        non_visited_paths = robot.intersections[position].get_possible_paths() - robot.intersections[position].get_visited_paths()
        
        available = non_visited_paths.pop()
        robot.intersections[position].add_visited_path(available)
        print("Taking direction", available)

        if left_direction(direction) == available:
            robot.intention = Rotate(True, direction)
            
        elif right_direction(direction) == available:
            robot.intention = Rotate(False, direction)

        else:
            robot.intention = MoveForward()
            

class Rotate(Intention):

    def __init__(self, left: bool, starting_direction: Direction):
        super().__init__()
        self.left = left
        self.starting_direction = starting_direction
        self.count = 0
    
    def act(self, robot: 'RobC2'):
        self.log_measured(robot)

        if self.count < 4:
            robot.driveMotors(self.velocity, self.velocity)
            self.count += 1
        else:
            robot.driveMotors(self.velocity if not self.left else -self.velocity, self.velocity if self.left else -self.velocity)

        if self.getDirection(robot.measures) != self.starting_direction and abs(self.get_angle_to_track(robot.measures)) < 30:

            # Update visited path
            direction = self.getDirection(robot.measures)        
            current_intersection = robot.current_intersection
            robot.intersections[current_intersection].add_visited_path(direction)

            robot.intention = Wander()


class MoveForward(Intention):

    def act(self, robot: 'RobC2'):
        self.log_measured(robot)

        # Update visited path
        direction = self.getDirection(robot.measures)        
        current_intersection = robot.current_intersection
        robot.intersections[current_intersection].add_visited_path(direction)

        robot.driveMotors(self.velocity, self.velocity)

        count = 0
        for ls in robot.measures.lineSensor:
            if ls == '1':
                count += 1 
            elif ls == '0' and count > 0:
                break

        if count <= 3:
            robot.intention = Wander()


class TurnBack(Intention):

    def __init__(self, starting_direction: Direction):
        super().__init__()
        self.starting_direction = starting_direction
        self.count = 0

    def act(self, robot: 'RobC2'):
        self.log_measured(robot)

        if self.count < 3:
            robot.driveMotors(-self.velocity, -self.velocity)
            self.count += 1
        else:
            robot.driveMotors(self.velocity, -self.velocity)

        if self.getDirection(robot.measures) == opposite_direction(self.starting_direction) and abs(self.get_angle_to_track(robot.measures)) < 35:
            robot.intention = Wander()


class CalculatePath(Intention):

    def __init__(self, start: Tuple[int, int], end: Tuple[int, int]):
        super().__init__()
        self.start = start
        self.end = end
        self.path = None
        self.moves = None
        self.current_move = None

    def act(self, robot: 'RobC2'):
        self.log_measured(robot)

        # First time calculating the path
        if self.path is None:
            self.path = self.find_path(robot.intersections, self.start, self.end)

        # First time calculating moves
        if self.moves is None:
            direction = self.getDirection(robot.measures) 

            self.moves = []
            self.calculate_moves(direction)

        # print("Path:", self.path)
        # print("Moves:", self.moves)

        if self.path:
            
            position = self.round_pos_to_intersection(robot.measures.x, robot.measures.y, robot)
                
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
                robot.driveMotors(self.velocity, self.velocity)

            # If the robot reached the end
            if position == self.end:
                robot.intention = Wander()

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