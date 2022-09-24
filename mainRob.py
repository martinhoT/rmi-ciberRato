
from lib2to3.pgen2.token import LEFTSHIFT
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, rob_name, rob_id, angles, host):
        CRobLinkAngs.__init__(self, rob_name, rob_id, angles, host)
        self.history = []

    # In this map the center of cell (i,j), (i in 0..6, j in 0..13) is mapped to labMap[i*2][j*2].
    # to know if there is a wall on top of cell(i,j) (i in 0..5), check if the value of labMap[i*2+1][j*2] is space or not
    def setMap(self, labMap):
        self.labMap = labMap

    def printMap(self):
        for l in reversed(self.labMap):
            print(''.join([str(l) for l in l]))

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
                self.wander()
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
                self.wander()
            
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

    '''
    # Score: 100 points
    def wander(self):
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
    def wander(self):
        
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
        '''

    # Score: (At least) 100 points
    def wander(self):

        left = self.measures.lineSensor[:3].count("1")
        right = self.measures.lineSensor[4:].count("1")

        print(self.measures.lineSensor)

        # Check history
        if self.history:
            action = self.history.pop(0)
            print('Rotate (' + str(action[0]) + ", " + str(action[1]) + ")")
            self.driveMotors(action[0], action[1])

        elif left - right > 0:
            print('Rotate left')
            self.driveMotors(-0.03, +0.03)
            # self.history = [(-0.1, +0.1) for _ in range((right - left)-1)]

        elif left - right < 0:
            print('Rotate right')
            self.driveMotors(+0.03, -0.03)
            # self.history = [(+0.1, -0.1) for _ in range(abs(right - left)-1)]

        else: 
            print('Go')
            self.driveMotors(0.1, 0.1)


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

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
