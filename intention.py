from typing import Union, TYPE_CHECKING
if TYPE_CHECKING:
    from robC1 import MyRob as RobC1
    from robC2 import MyRob as RobC2

from directions import Direction, DIRECTIONS_ARRAY

class Intention:

    def __init__(self):
        self.velocity = 0.8

    def act(self, robot: Union['RobC1', 'RobC2']):
        raise NotImplementedError()


class Wander(Intention):

    def act(self, robot: 'RobC2'):

        # robot.history = 0 -> Straight
        # robot.history = 1 -> Left
        # robot.history = 2 -> Right
        
        print(robot.measures.lineSensor)
        n_active = robot.measures.lineSensor.count("1")

        # Robot is off track
        if (n_active == 0):
            
            print('Off track - Backtracking...')

            backtrack = self.velocity
            last_move = robot.history.pop()
            if last_move == 0:
                robot.driveMotors(-backtrack, -backtrack)
            elif last_move == 1:
                robot.driveMotors(backtrack, -backtrack)
            else: 
                robot.driveMotors(-backtrack, backtrack)
            
            return

        direction = robot.getDirection()
        angle_to_track = robot.measures.compass - \
                 (90 if direction == Direction.N
            else -90 if direction == Direction.S
            else  180 if robot.measures.compass > 135
            else -180 if robot.measures.compass < -135
            else 0)
        print('Angle to track:', angle_to_track)

        # Robot is on track
        left = robot.measures.lineSensor[:3].count("1")
        right = robot.measures.lineSensor[4:].count("1")

        leftTurn = left == 3
        rightTurn = right == 3

        if leftTurn or rightTurn:

            # Obtain  position of robot in the map
            x = round(robot.measures.x)
            y = round(robot.measures.y)
            position = (x, y)

            if position not in robot.intersections:

                robot.intersections[position] = []

                if leftTurn:
                    robot.intersections[position].append(DIRECTIONS_ARRAY[(direction.value - 1) % 4])
                if rightTurn:
                    robot.intersections[position].append(DIRECTIONS_ARRAY[(direction.value + 1) % 4])
            
                robot.intention = CheckIntersectionForward()
            
                # robot.driveMotors(self.velocity, self.velocity)

            else:
                
                if not robot.intersections[position]:
                    # TODO: Input
                    pass

                else:
                    robot.intention = TurnIntersection()
        
        if left - right > 1:
            print('Rotate left')
            robot.driveMotors(-self.velocity, +self.velocity)
            robot.history.append(1)

        elif left - right < -1:
            print('Rotate right')
            robot.driveMotors(+self.velocity, -self.velocity)
            robot.history.append(2)

        else: 
            print('Go')
            action = robot.safeguard()
            robot.driveMotors(action[0], action[1])
            robot.history.append(0)

            print("Compass:", robot.measures.compass)
            print("Direction:", robot.getDirection())
            
            # Update map
            x = round(robot.measures.x)
            y = round(robot.measures.y)
            if (x,y) not in robot.map:
                robot.map[(x,y)] = robot.getDirection()
                # print(robot.map)

        # Move one line segment <=> Move 2 cells
        # print("X:", robot.measures.x)
        # print("Y:", robot.measures.y)

class CheckIntersectionForward(Intention):

    def act(self, robot: 'RobC2'):
        robot.driveMotors(self.velocity, self.velocity)
        
        if all(ls == '0' for ls in robot.measures.lineSensor):
            robot.intention = CheckIntersectionBacktrack()

        # Obtain  position of robot in the map
        x = round(robot.measures.x)
        y = round(robot.measures.y)
        position = (x, y)
        direction = robot.getDirection()

        robot.intersections[position].append(direction)
        robot.intention = CheckIntersectionBacktrack()

class CheckIntersectionBacktrack(Intention):

    def act(self, robot: 'RobC2'):
        robot.driveMotors(-self.velocity, -self.velocity)
        robot.intention = TurnIntersection()


class TurnIntersection(Intention):

    def act(self, robot: 'RobC2'):
        
        x = round(robot.measures.x)
        y = round(robot.measures.y)
        position = (x, y)
        direction = robot.getDirection()

        for available in robot.intersections[position]:

            if (direction.value - 1) % 4 == available.value:
                print("Rotate left")
                robot.intersections[position].remove(available)
                robot.intention = Rotate(True, direction)
                
            elif (direction.value + 1) % 4 == available.value:
                print("Rotate right")
                robot.intersections[position].remove(available)
                robot.intention = Rotate(False, direction)

            else:
                print("Go")
                robot.intersections[position].remove(direction)
                robot.driveMotors(self.velocity, self.velocity) # Leave intersection
                robot.intention = Wander()
        

class Rotate(Intention):

    def __init__(self, left: bool, starting_direction: Direction):
        super().__init__()
        self.velocity = self.velocity * (-1)**int(left)
        self.starting_direction = starting_direction
    
    def act(self, robot: 'RobC2'):
        robot.driveMotors(-self.velocity, self.velocity)
        if robot.getDirection() != self.starting_direction:
            robot.intention = Wander()
