
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from directions import opposite_direction

from intention import Finish, Rotate
from utils import get_direction, map_to_text
from robData import RobData

CELLROWS=7
CELLCOLS=14

class MyRob(CRobLinkAngs):
    def __init__(self, robName, rob_id, angles, host, fname='robC2'):
        CRobLinkAngs.__init__(self, robName, rob_id, angles, host)
        
        # All known intersections have been exhausted, which should mean that the entire map has been traversed
        def exhausted_intersections(rdata: RobData) -> bool:
            return len(rdata.intersections) != 0 \
                and all(len(i.get_possible_paths() - i.get_visited_paths()) == 0 for i in rdata.intersections.values())

        self.data = RobData(
            finish_condition=exhausted_intersections
        )
        self.intention = None
        self.fname = fname

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

            # Initial intention setup
            if not self.intention:
                self.data.starting_position = (self.measures.x, self.measures.y)
                direction = get_direction(self.measures.compass)
                self.intention = Rotate(
                    starting_direction=direction,
                    end_direction=opposite_direction(direction),
                    advancement_steps=15)

            if self.measures.endLed:
                print(self.robName + " exiting")
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
                motors, next_intention = self.intention.act(self.measures, self.data)
                if motors:
                    self.driveMotors(*motors)
                if isinstance(self.intention, Finish):
                    break
                if next_intention:
                    self.intention = next_intention
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
                motors, next_intention = self.intention.act(self.measures, self.data)
                if motors:
                    self.driveMotors(*motors)
                if isinstance(self.intention, Finish):
                    break
                if next_intention:
                    self.intention = next_intention

        # Save final map
        with open(self.fname + ".map", "w") as file:
            for line in map_to_text(self.data.pmap):
                print(''.join(line), file=file)
        
        self.finish()

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
fname = 'solution'

for i in range(1, len(sys.argv),2):
    if (sys.argv[i] == "--host" or sys.argv[i] == "-h") and i != len(sys.argv) - 1:
        host = sys.argv[i + 1]
    elif (sys.argv[i] == "--pos" or sys.argv[i] == "-p") and i != len(sys.argv) - 1:
        pos = int(sys.argv[i + 1])
    elif (sys.argv[i] == "--robname" or sys.argv[i] == "-r") and i != len(sys.argv) - 1:
        rob_name = sys.argv[i + 1]
    elif (sys.argv[i] == "--map" or sys.argv[i] == "-m") and i != len(sys.argv) - 1:
        mapc = Map(sys.argv[i + 1])
    elif (sys.argv[i] == "--filename" or sys.argv[i] == "-f") and i != len(sys.argv) -1:
        fname = sys.argv[i + 1]
    else:
        print("Unkown argument", sys.argv[i])
        quit()

if __name__ == '__main__':
    rob=MyRob(rob_name,pos,[0.0,60.0,-60.0,180.0],host,fname)
    if mapc != None:
        rob.setMap(mapc.labMap)
        rob.printMap()
    
    rob.run()