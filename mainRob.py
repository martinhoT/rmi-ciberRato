
from lib2to3.pgen2.token import LEFTSHIFT
from shutil import SpecialFileError
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host, approach='base', lineSensorMemoryN=7):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.history = []
        self.memory = {
            'lineSensor': [ ['0']*7 ]*lineSensorMemoryN
        }
        self.wander = self._wanderApproaches[approach]

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))
            
    '''
        Line sensor:
        o _
        o _|-> 0.08
        o
        o
        o
        o
        o
        Line width: 0.2
        Distance to robot's center: 0.438
        Max speed: 0.15
    '''

    # Score: 100 points
    def wanderBasic(self):
        print('Line sensors:', self.measures.lineSensor)

        if self.measures.lineSensor[-1] == '1':
            print('Rotate right')
            self.driveMotors(0.1, -0.1)
        elif self.measures.lineSensor[0] == '1':
            print('Rotate left')
            self.driveMotors(-0.1, 0.1)
        elif self.measures.lineSensor[-2] == '1':
            print('Rotate slowly right')
            self.driveMotors(0.1, 0.0)
        elif self.measures.lineSensor[1] == '1':
            print('Rotate slowly left')
            self.driveMotors(0.0, 0.1)
        else:
            print('Go')
            self.driveMotors(0.1, 0.1)

    # Score: Robot crashes into a wall
    def wanderByActionHistory(self):
        
        # print('Line sensors:', self.measures.lineSensor)

        left = self.measures.lineSensor[:3].count("1")
        right = self.measures.lineSensor[4:].count("1")

        # Check history
        if self.history:
            action = self.history.pop(0)
            print('Rotate (' + str(action[0]) + ", " + str(action[1]) + ")")
            self.driveMotors(action[0], action[1])
        
        # Check line sensors
        # High detour
        elif left >= 2 or right >= 2:

            # Small difference
            if left - right == 1:
                print('Rotate slightly to the left')
                self.driveMotors(-0.01, +0.01)
            if left - right == -1:
                print('Rotate slightly to the right')
                self.driveMotors(+0.01, -0.01)

            # High difference
            if left - right > 1:
                print('Rotate left')
                self.driveMotors(-0.05, +0.05)
                self.history = [(-0.1, +0.1) for _ in range(right - left-1)]
            elif left - right < -1:
                print('Rotate right')
                self.driveMotors(+0.05, -0.05)
                self.history = [(+0.1, -0.1) for _ in range(abs(right - left)-1)]

            # No difference
            else: 
                print('Go')
                self.driveMotors(0.1, 0.1)
        
        # Small detour
        else: 
            print('Go')
            self.driveMotors(0.1, 0.1)

    # Score: (At least) 100 points
    def wanderByLineSensorMemory(self):
        """
Wander using the last X `lineSensor` values. These values are only used to calculate the desired turn speed.

Example for the last 3 `lineSensor` values, from most recent to least recent:
```
[
    [0,  0,  1,  1,  1,  1,  1],
    [0,  0,  1,  1,  1,  0,  0],
    [0,  0,  1,  1,  1,  0,  0],
]
```
This 3x7 matrix is converted to a single aggregate list, by treating each column as a binary number.
This results in the most recent values having greater impact in the current turn speed than older ones.

The aggregate list is used to determine the desired turn speed to the left and to the right.
        """

        self.memory['lineSensor'] = [self.measures.lineSensor] + self.memory['lineSensor'][:-1]
        
        lineSensorAgg = [int(''.join(list(bins)), 2) for bins in zip(*self.memory['lineSensor'])]

        if not any(lineSensorAgg):
            print('OFF TRACK')
            return

        print('Line sensors:', self.measures.lineSensor, 'Agg:', lineSensorAgg, 'Ground:', self.measures.ground)

        # Ideally, the turn scales are < 1.0
        maxTurnScale = 3 * (2**7-1)
        leftTurnScale = sum(lineSensorAgg[:3][::-1]) / maxTurnScale
        rightTurnScale = sum(lineSensorAgg[4:]) / maxTurnScale
        
        left = self.measures.lineSensor[:3].count("1")
        right = self.measures.lineSensor[4:].count("1")

        # In a turn, the inner motors have a squared turn speed so that sharper turns 
        # are only applied when the turnScale is large enough (close to 1.0)
        if left > right:
            sharpTurnScale = leftTurnScale**2 if leftTurnScale > 0.5 else 0.0
            print('Rotate left', leftTurnScale)
            self.driveMotors(-0.1 * sharpTurnScale, +0.15 * leftTurnScale)

        elif left < right:
            sharpTurnScale = rightTurnScale**2 if rightTurnScale > 0.5 else 0.0
            print('Rotate right', rightTurnScale)
            self.driveMotors(+0.15 * rightTurnScale, -0.1 * sharpTurnScale)

        else:
            print('Go')
            action = self.safeguard()
            self.driveMotors(action[0], action[1])

    # Score: (At least) 100 points
    def wanderBase(self):

        left = self.measures.lineSensor[:3].count("1")
        right = self.measures.lineSensor[4:].count("1")
        
        if left - right > 0:
            print('Rotate left')
            self.driveMotors(-0.03, +0.03)

        elif left - right < 0:
            print('Rotate right')
            self.driveMotors(+0.03, -0.03)

        else: 
            print('Go')
            action = self.safeguard()
            self.driveMotors(action[0], action[1])

    """
    count = 0
    def spin(self):

        print('Rotate right')
        self.driveMotors(0.15, -0.15)
        self.count += 1
        print("Count:", self.count)
    """

    def wanderWithRotationHistory(self):

        left = self.measures.lineSensor[:3].count("1")
        right = self.measures.lineSensor[4:].count("1")

        print(self.measures.lineSensor)

        # Check history
        if self.history:
            action = self.history.pop(0)
            print('Rotate (' + str(action[0]) + ", " + str(action[1]) + ")")
            self.driveMotors(action[0], action[1])

        elif right == 3:
            print('Rotate right')
            self.driveMotors(0.15, 0.15)
            self.history = [(0.15, 0.15)] + [(0.15, -0.15)]*5 + [(0.15, 0.15)]
        
        elif left == 3:
            print('Rotate left')
            self.driveMotors(0.15, 0.15)
            self.history = [(0.15, 0.15)] + [(-0.15, 0.15)]*5 + [(0.15, 0.15)]

        else: 
            self.wanderBase()
        

    def safeguard(self):
        center_id = 0
        left_id = 1
        right_id = 2
        back_id = 3

        if self.measures.irSensor[center_id] > 5.0 \
           or self.measures.irSensor[left_id]  > 5.0 \
           or self.measures.irSensor[right_id] > 5.0 \
           or self.measures.irSensor[back_id]  > 5.0:
            return (-0.1, +0.1)
        elif self.measures.irSensor[left_id]> 2.7:
            return (0.1, 0.0)
        elif self.measures.irSensor[right_id]> 2.7:
            return (0.0, 0.1)
        return (0.1, 0.1)

    _wanderApproaches = {
        'base': wanderBase,
        'basic': wanderBasic,
        'byActionHistory': wanderByActionHistory,
        'byLineSensorMemory': wanderByLineSensorMemory,
        'withRotationHistory': wanderWithRotationHistory,
        'spin': spin
    }

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            if self.measures.endLed:
                print(self.rob_name + " exiting")
                quit()

            if state == 'stop' and self.measures.start:
                state = stopped_state

            if state != 'stop' and self.measures.stop:
                stopped_state = state
                state = 'stop'

            if state == 'run':
                if self.measures.visitingLed==True:
                    state='wait'
                if self.measures.ground==0:
                    self.setVisitingLed(True);
                self.wander(self)
            elif state=='wait':
                self.setReturningLed(True)
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    state='return'
                self.driveMotors(0.0,0.0)
            elif state=='return':
                if self.measures.visitingLed==True:
                    self.setVisitingLed(False)
                if self.measures.returningLed==True:
                    self.setReturningLed(False)
                self.wander(self)


class Map():
    def __init__(self, filename):
        tree = ET.parse(filename)
        root = tree.getroot()
        
        self.labMap = [[' '] * (CELLCOLS*2-1) for i in range(CELLROWS*2-1) ]
        i=1
        for child in root.iter('Row'):
           line=child.attrib['Pattern']
           row =int(child.attrib['Pos'])
           if row % 2 == 0:  # this line defines vertical lines
               for c in range(len(line)):
                   if (c+1) % 3 == 0:
                       if line[c] == '|':
                           self.labMap[row][(c+1)//3*2-1]='|'
                       else:
                           None
           else:  # this line defines horizontal lines
               for c in range(len(line)):
                   if c % 3 == 0:
                       if line[c] == '-':
                           self.labMap[row][c//3*2]='-'
                       else:
                           None
               
           i=i+1


rob_name = "pClient1"
host = "localhost"
pos = 1
mapc = None
approach = 'base'

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--approach" or sys.argv[i] == "-a") and i != len(sys.argv) - 1:
        approach = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host,approach)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
