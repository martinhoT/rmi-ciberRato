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
LOG_STARTING_POS = False
LOG_INTENTION = True
LOG_SENSORS = True
LOG_INTERSECTIONS = True
LOG_MAP = True

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
                # print(position, "- Possible:" , intersection.get_possible_paths(), "- Visited:", intersection.get_visited_paths())
                print(position, "- Not Visited:" , intersection.get_possible_paths() - intersection.get_visited_paths(), "- Neighbours:", intersection.get_neighbours())
            """
            for intersection, (possible_directions, non_visited_directions) in robot.intersections.items():
                if non_visited_directions:
                    print(intersection, non_visited_directions)
            """
        if LOG_MAP and robot.map:
            for line in map_to_text(list(robot.map.keys())):
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

        return (self.velocity * (1 - (-angle_to_track/45) - (left*1/3 if left >= 2 else 0)), 
                self.velocity * (1 - (angle_to_track/45) - (right*1/3 if right >= 2 else 0)))

    def get_angle_to_track(self, measures: CMeasures):
        direction = self.getDirection(measures)
        return measures.compass - \
                 (90 if direction == Direction.N
            else -90 if direction == Direction.S
            else  180 if measures.compass > 135
            else -180 if measures.compass < -135
            else 0)

    def adjust_position_to_intersections(self, position, intersections):
        if position not in intersections:
            for i in range(-1, 2):
                for j in range(-1, 2):
                    if (position[0] + i, position[1] + j) in intersections:
                        return (position[0] + i, position[1] + j)
            return None
        return position


class Wander(Intention):

    def act(self, robot: 'RobC2'):
        self.log_measured(robot)

        # robot.history = 0 -> Straight
        # robot.history = 1 -> Left
        # robot.history = 2 -> Right

        if not robot.starting_position:
            robot.starting_position = (robot.measures.x, robot.measures.y)

        n_active = robot.measures.lineSensor.count("1")

        x, y = self.round_pos(robot.measures.x, robot.measures.y, robot)
        if (x,y) not in robot.map:
            robot.map[(x,y)] = self.getDirection(robot.measures)
            # print(robot.map)

        # Robot is off track
        if (n_active == 0):
            robot.driveMotors(0.0, 0.0)
            return
            
            # print('Off track - Backtracking...')

            backtrack = self.velocity
            last_move = robot.history.pop()
            if last_move == 0:
                robot.driveMotors(-backtrack, -backtrack)
            elif last_move == 1:
                robot.driveMotors(backtrack, -backtrack)
            else: 
                robot.driveMotors(-backtrack, backtrack)
            
            return

        # Robot is on track
        left = robot.measures.lineSensor[:3].count("1")
        right = robot.measures.lineSensor[4:].count("1")

        leftTurn = left == 3
        rightTurn = right == 3

        # Possible intersection found
        if (leftTurn or rightTurn) and robot.measures.lineSensor[3] == '1':

            # Obtain position of robot in the map
            position = self.round_pos(robot.measures.x, robot.measures.y, robot)
            direction = self.getDirection(robot.measures)

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

                    # TODO: choose where to go if intersection is exhausted

                    if leftTurn:
                        robot.intention = Rotate(True, self.getDirection(robot.measures))
                    elif rightTurn:
                        robot.intention = Rotate(False, self.getDirection(robot.measures))

                else:
                    robot.intention = TurnIntersection()
        
        robot.driveMotors(*self.follow_path(robot.measures))
        return

        if left - right > 1:
            # print('Rotate left')
            robot.driveMotors(-self.velocity, +self.velocity)
            robot.history.append(1)

        elif left - right < -1:
            # print('Rotate right')
            robot.driveMotors(+self.velocity, -self.velocity)
            robot.history.append(2)

        else: 
            # print('Go')
            action = self.safeguard(robot.measures)
            robot.driveMotors(action[0], action[1])
            robot.history.append(0)

            # print("Compass:", robot.measures.compass)
            # print("Direction:", self.getDirection(robot.measures))
            
            # Update map
            x = self.round_pos(robot.measures.x, robot)
            y = self.round_pos(robot.measures.y, robot)
            if (x,y) not in robot.map:
                robot.map[(x,y)] = self.getDirection(robot.measures)
                # print(robot.map)

        # Move one line segment <=> Move 2 cells
        # print("X:", robot.measures.x)
        # print("Y:", robot.measures.y)


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

        position = (robot.measures.x, robot.measures.y)
        direction = self.getDirection(robot.measures)

        # If the robot is back at the intersection
        if all(ls == '1' for ls in robot.measures.lineSensor[:3]) or all(ls == '1' for ls in robot.measures.lineSensor[4:]):
            
            if self.has_intersection_forward:
                # position = self.adjust_position_to_intersections(position, robot.intersections)

                """
                position = self.round_pos(*position, robot)

                # Should not happen
                if position not in robot.intersections:
                    print('Position', position, 'is not in the intersections array, but should be! Intersections array:', robot.intersections)

                print('NEW INTERSECTION at', position, 'in direction', direction)
                robot.intersections[position].add_path(direction)

                """
                current_intersection = robot.current_intersection
                robot.intersections[current_intersection].add_path(direction)

            robot.driveMotors(self.velocity, self.velocity)
            robot.intention = TurnIntersection()

        else:
            robot.driveMotors(-self.velocity, -self.velocity)


class TurnIntersection(Intention):

    def act(self, robot: 'RobC2'):
        
        x, y = self.round_pos(robot.measures.x, robot.measures.y, robot)
        position = self.adjust_position_to_intersections((x, y), robot.intersections)
        direction = self.getDirection(robot.measures)

        robot.driveMotors(0.0, 0.0)

        print('About to turn, possible directions:', robot.intersections[position].get_possible_paths())
        non_visited_paths = robot.intersections[position].get_possible_paths() - robot.intersections[position].get_visited_paths()
        
        available = non_visited_paths.pop()
        robot.intersections[position].add_visited_path(available)
        print("Taking direction", available)

        if (direction.value - 1) % 4 == available.value:
            robot.intention = Rotate(True, direction)
            
        elif (direction.value + 1) % 4 == available.value:
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

