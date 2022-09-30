
from lib2to3.pgen2.token import LEFTSHIFT
from shutil import SpecialFileError
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET

CELLROWS=7
CELLCOLS=14

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
    Diameter: 0.5
    Cell width: 1.0
    Max speed: 0.15

    GPS:
    - no noise
    - random starting position
'''

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

    # Score: 5550
    def wander(self):

        # self.history = 0 -> Straight
        # self.history = 1 -> Left
        # self.history = 2 -> Right
        
        print(self.measures.lineSensor)
        n_active = self.measures.lineSensor.count("1")

        # Robot is off track
        if (n_active == 0):
            
            print('Off track - Backtracking...')

            backtrack = 0.15
            last_move = self.history.pop(0)
            if last_move == 0:
                self.driveMotors(-backtrack, -backtrack)
            elif last_move == 1:
                self.driveMotors(backtrack, -backtrack)
            else: 
                self.driveMotors(-backtrack, backtrack)
            
            return

        # Robot is on track
        left = self.measures.lineSensor[:3].count("1")
        right = self.measures.lineSensor[4:].count("1")
        
        if left - right > 1:
            print('Rotate left')
            self.driveMotors(-0.15, +0.15)
            self.history.append(1)

        elif left - right < -1:
            print('Rotate right')
            self.driveMotors(+0.15, -0.15)
            self.history.append(2)

        else: 
            print('Go')
            action = self.safeguard()
            self.driveMotors(action[0], action[1])
            self.history.append(0)

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
        return (0.15, 0.15) # Max speed

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
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()
