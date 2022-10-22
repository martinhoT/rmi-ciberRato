import math

from os import system
from typing import Tuple, Union, TYPE_CHECKING
from croblink import CMeasures
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
            for intersection, (possible_directions, non_visited_directions) in robot.intersections.items():
                if non_visited_directions:
                    print(intersection, non_visited_directions)
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

    def round_pos_to_intersection(self, x: float, y: float, robot: Union['RobC1', 'RobC2']):
        if not robot.starting_position:
            return 0, 0
        return round((x-robot.starting_position[0])/2)*2, round((y-robot.starting_position[1])/2)*2


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

            if position not in robot.intersections:

                robot.intersections[position] = (set(), [])
                
                robot.intersections[position][0].add( opposite_direction(direction) )
                if leftTurn:
                    print('NEW INTERSECTION at', position, 'in direction', left_direction(direction))
                    robot.intersections[position][0].add( left_direction(direction) )
                    robot.intersections[position][1].append( left_direction(direction) )
                if rightTurn:
                    print('NEW INTERSECTION at', position, 'in direction', right_direction(direction))
                    robot.intersections[position][0].add( right_direction(direction) )
                    robot.intersections[position][1].append( right_direction(direction) )
            
                robot.intention = CheckIntersectionForward(position)
                robot.driveMotors(0.0, 0.0)
                return

            else:
                if opposite_direction(direction) in robot.intersections[position][1]:
                    robot.intersections[position][1].remove( opposite_direction(direction) )

                if not robot.intersections[position][1]:
                    # TODO: choose where to go if intersection is exhausted
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
        if distance_of_intersections_in_front_of_me:
            # There needs to be straight path to the intersection
            closest_distance = min(distance_of_intersections_in_front_of_me)
            # TODO: simplify like above?
            if direction == Direction.E:
                positions_to_be_covered = {(position[0] + i, position[1]) for i in range(1, closest_distance + 1)}
            elif direction == Direction.W:
                positions_to_be_covered = {(position[0] - i, position[1]) for i in range(1, closest_distance + 1)}
            elif direction == Direction.N:
                positions_to_be_covered = {(position[0], position[1] + i) for i in range(1, closest_distance + 1)}
            elif direction == Direction.S:
                positions_to_be_covered = {(position[0], position[1] - i) for i in range(1, closest_distance + 1)}
            
            # Should reach that intersection in a known straight path from this position
            if len(positions_to_be_covered - set(robot.map)) == 0:
                extra_velocity = (closest_distance/4)**2 if closest_distance/4 < 1 else 1
            
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

        position = (robot.measures.x, robot.measures.y)
        direction = self.getDirection(robot.measures)

        if all(ls == '1' for ls in robot.measures.lineSensor[:3]) or all(ls == '1' for ls in robot.measures.lineSensor[4:]):
            if self.has_intersection_forward:
                position = self.round_pos_to_intersection(*position, robot)
                # Should not happen
                if position not in robot.intersections:
                    print('Position', position, 'is not in the intersections array, but should be! Intersections array:', robot.intersections)

                print('NEW INTERSECTION at', position, 'in direction', direction)
                robot.intersections[position][0].add(direction)
                robot.intersections[position][1].append(direction)
            robot.driveMotors(self.velocity, self.velocity)
            robot.intention = TurnIntersection()
        else:
            robot.driveMotors(-self.velocity, -self.velocity)


class TurnIntersection(Intention):

    def act(self, robot: 'RobC2'):
        
        position = self.round_pos_to_intersection(robot.measures.x, robot.measures.y, robot)
        direction = self.getDirection(robot.measures)

        robot.driveMotors(0.0, 0.0)

        print('About to turn, possible directions:', robot.intersections[position])
        available = robot.intersections[position][1].pop()

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


class TurnBack(Intention):

    def __init__(self, starting_direction: Direction):
        super().__init__()
        self.starting_direction = starting_direction
        self.count = 0

    def act(self, robot: 'RobC2'):
        self.log_measured(robot)

        if self.count < 4:
            robot.driveMotors(-self.velocity, -self.velocity)
            self.count += 1
        else:
            robot.driveMotors(self.velocity, -self.velocity)

        if self.getDirection(robot.measures) == opposite_direction(self.starting_direction) and abs(self.get_angle_to_track(robot.measures)) < 35:
            robot.intention = Wander()
