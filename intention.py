import math
from typing import Tuple, Union, TYPE_CHECKING
from croblink import CMeasures
if TYPE_CHECKING:
    from robC1 import MyRob as RobC1
    from robC2 import MyRob as RobC2

from directions import Direction, DIRECTIONS_ARRAY


class Intention:

    def __init__(self):
        self.velocity = 0.08
        print(self.__class__.__name__, 'instanced')

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
    
    def round_pos(self, coord: float):
        return math.floor(coord + 0.5)

    def log_measured(self, robot: Union['RobC1', 'RobC2']):
        print(f'{robot.measures.lineSensor} ({robot.measures.x}, {robot.measures.y}) -> ({self.round_pos(robot.measures.x)}, {self.round_pos(robot.measures.y)})')

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
        for i in range(-1, 2):
            for j in range(-1, 2):
                if (position[0] + i, position[1] + j) in intersections:
                    return (position[0] + i, position[1] + j)


class Wander(Intention):

    def act(self, robot: 'RobC2'):
        self.log_measured(robot)

        # robot.history = 0 -> Straight
        # robot.history = 1 -> Left
        # robot.history = 2 -> Right

        n_active = robot.measures.lineSensor.count("1")

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

            # Obtain  position of robot in the map
            x = self.round_pos(robot.measures.x)
            y = self.round_pos(robot.measures.y)
            position = (x, y)

            if position not in robot.intersections:

                robot.intersections[position] = []
                direction = self.getDirection(robot.measures)

                if leftTurn:
                    print('NEW INTERSECTION at', position, 'in direction', DIRECTIONS_ARRAY[(direction.value - 1) % 4])
                    robot.intersections[position].append(DIRECTIONS_ARRAY[(direction.value - 1) % 4])
                if rightTurn:
                    print('NEW INTERSECTION at', position, 'in direction', DIRECTIONS_ARRAY[(direction.value + 1) % 4])
                    robot.intersections[position].append(DIRECTIONS_ARRAY[(direction.value + 1) % 4])
            
                robot.intention = CheckIntersectionForward(position)
                robot.driveMotors(0.0, 0.0)
                return

            else:
                if not robot.intersections[position]:
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
            x = self.round_pos(robot.measures.x)
            y = self.round_pos(robot.measures.y)
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

        # Obtain  position of robot in the map
        # x = self.round_pos(robot.measures.x)
        # y = self.round_pos(robot.measures.y)
        # position = (x, y)
        # direction = self.getDirection(robot.measures)

        # robot.intersections[position].append(direction)
        # robot.intention = CheckIntersectionBacktrack()

class CheckIntersectionForwardBacktrack(Intention):

    def __init__(self, intersection_pos: Tuple[int, int], has_intersection_forward: bool):
        super().__init__()
        self.intersection_pos = intersection_pos
        self.has_intersection_forward = has_intersection_forward

    def act(self, robot: 'RobC2'):
        self.log_measured(robot)

        x = self.round_pos(robot.measures.x)
        y = self.round_pos(robot.measures.y)
        position = (x, y)
        direction = self.getDirection(robot.measures)

        if all(ls == '1' for ls in robot.measures.lineSensor[:3]) or all(ls == '1' for ls in robot.measures.lineSensor[4:]):
            if self.has_intersection_forward:
                position = self.adjust_position_to_intersections(position, robot.intersections)
                # Should not happen
                if position not in robot.intersections:
                    print('Position', position, 'is not in the intersections array, but should be! Intersections array:', robot.intersections)

                print('NEW INTERSECTION at', position, 'in direction', direction)
                robot.intersections[position].append(direction)
            robot.driveMotors(self.velocity, self.velocity)
            robot.intention = TurnIntersection()
        else:
            robot.driveMotors(-self.velocity, -self.velocity)


class TurnIntersection(Intention):

    def act(self, robot: 'RobC2'):
        
        x = self.round_pos(robot.measures.x)
        y = self.round_pos(robot.measures.y)
        position = self.adjust_position_to_intersections((x, y), robot.intersections)
        direction = self.getDirection(robot.measures)

        robot.driveMotors(0.0, 0.0)

        print('About to turn, possible directions:', robot.intersections[position])
        for available in robot.intersections[position]:

            if (direction.value - 1) % 4 == available.value:
                robot.intersections[position].remove(available)
                robot.intention = Rotate(True, direction)
                
            elif (direction.value + 1) % 4 == available.value:
                robot.intersections[position].remove(available)
                robot.intention = Rotate(False, direction)

            else:
                robot.intersections[position].remove(direction)
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

        if self.getDirection(robot.measures) != self.starting_direction and abs(self.get_angle_to_track(robot.measures)) < 35:
            robot.intention = Wander()

class MoveForward(Intention):

    def act(self, robot: 'RobC2'):
        self.log_measured(robot)

        robot.driveMotors(self.velocity, self.velocity)

        count = 0
        for ls in robot.measures.lineSensor:
            if ls == '1':
                count += 1 
            elif ls == '0' and count > 0:
                break

        if count <= 3:
            robot.intention = Wander()
