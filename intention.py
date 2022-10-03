from typing import Union, TYPE_CHECKING
if TYPE_CHECKING:
    from robC1 import MyRob as RobC1
    from robC2 import MyRob as RobC2


class Intention:

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

            backtrack = 0.15
            last_move = robot.history.pop(0)
            if last_move == 0:
                robot.driveMotors(-backtrack, -backtrack)
            elif last_move == 1:
                robot.driveMotors(backtrack, -backtrack)
            else: 
                robot.driveMotors(-backtrack, backtrack)
            
            return

        orientation = robot.getOrientation()
        angle_to_track = robot.measures.compass - \
                 (90 if orientation == 'N'
            else -90 if orientation == 'S'
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
            robot.intention = CheckIntersectionForward()
            robot.driveMotors(0.1, 0.1)
        
        if left - right > 1:
            print('Rotate left')
            robot.driveMotors(-0.15, +0.15)
            robot.history.append(1)

        elif left - right < -1:
            print('Rotate right')
            robot.driveMotors(+0.15, -0.15)
            robot.history.append(2)

        else: 
            print('Go')
            action = robot.safeguard()
            robot.driveMotors(action[0], action[1])
            robot.history.append(0)

            print("Compass:", robot.measures.compass)
            print("Orientation:", robot.getOrientation())
            
            # Update map
            x = round(robot.measures.x)
            y = round(robot.measures.y)
            if (x,y) not in robot.map:
                robot.map[(x,y)] = robot.getOrientation()
                print(robot.map)

        # Move one line segment <=> Move 2 cells
        print("X:", robot.measures.x)
        print("Y:", robot.measures.y)

class CheckIntersectionForward(Intention):

    def act(self, robot: 'RobC2'):
        robot.driveMotors(0.1, 0.1)
        
        if all(ls == '0' for ls in robot.measures.lineSensor):
            robot.intention = CheckIntersectionBacktrack()

class CheckIntersectionBacktrack(Intention):

    def act(self, robot: 'RobC2'):
        robot.driveMotors(-0.1, -0.1)

