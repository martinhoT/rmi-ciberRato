import itertools
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from directions import opposite_direction

from graph import Checkpoint
from intention import Rotate, Finish, PrepareFinish
from utils import get_direction, map_to_text, wavefront_expansion, calculate_next_movement, update_checkpoints_neighbours
from robData import MovementData, RobData

CELLROWS=7
CELLCOLS=14

# TODO: test with custom maps containing large corridors (to check if movement guess is correct)
class MyRob(CRobLinkAngs):
    def __init__(self, robName, rob_id, angles, host, fname='robC2'):
        CRobLinkAngs.__init__(self, robName, rob_id, angles, host)
        
        def exhausted_intersections(rdata: RobData) -> bool:
            return len(rdata.intersections) != 0 \
                and all(len(i.get_possible_paths() - i.get_visited_paths()) == 0 for i in rdata.intersections.values()) \
                and len(rdata.intersections_intentions) == 0
        
        starting_position = (0, 0)
        # We assume the robot is always facing east
        starting_angle = 0.0
        self.data = RobData(
            starting_position=starting_position,
            finish_condition=exhausted_intersections,
            prepare_before_finish=True,
            movement_guess=MovementData((0, 0), starting_position, starting_angle)
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

    def pairwise(self, iterable):
        a, b = itertools.tee(iterable)
        next(b, None)
        return zip(a, b)

    def follow_intention(self) -> bool:
        motors, next_intention = self.intention.act(self.measures, self.data)
        
        if not motors:
            motors = self.data.previous_action
        self.data.previous_action = motors
        self.data.movement_guess = calculate_next_movement(motors, self.data.movement_guess)
        self.driveMotors(*motors)

        if isinstance(next_intention, PrepareFinish):
            update_checkpoints_neighbours(self.data)

        if isinstance(self.intention, Finish):
            return True

        if next_intention:
            self.intention = next_intention
        return False

    def run(self):
        if self.status != 0:
            print("Connection refused or error")
            quit()

        state = 'stop'
        stopped_state = 'run'

        while True:
            self.readSensors()

            # Intention initialization that requires sensor measures
            if not self.intention:
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
                to_finish = self.follow_intention()
                if to_finish:
                    break
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
                to_finish = self.follow_intention()
                if to_finish:
                    break

        # Save final map
        with open(self.fname + ".map", "w") as file:
            for line in map_to_text(self.data.pmap):
                print(''.join(line), file=file)

        # Obtain checkpoints
        checkpoints = list(self.data.checkpoints.values())

        # List to keep track of all possible paths
        all_path_positions = []

        for sequence in itertools.permutations(checkpoints[1:]):
            sequence = [checkpoints[0]] + list(sequence) + [checkpoints[0]]

            path = self.pairwise(sequence)
            path_positions = []
            
            for start_node, end_node in path:
                path_intersections, _ = wavefront_expansion(start_node, key=lambda node: isinstance(node, Checkpoint) and node.get_coordinates() == end_node.get_coordinates())
                
                for start, end in self.pairwise(path_intersections):

                    distance_x = end.get_x() - start.get_x()
                    distance_y = end.get_y() - start.get_y()

                    if distance_x > 0:
                        path_positions += [(start.get_x() + i, start.get_y()) for i in range(0, abs(distance_x), 2)]

                    elif distance_x < 0:
                        path_positions += [(start.get_x() - i, start.get_y()) for i in range(0, abs(distance_x), 2)]

                    elif distance_y > 0:
                        path_positions += [(start.get_x(), start.get_y() + i) for i in range(0, abs(distance_y), 2)]

                    elif distance_y < 0:
                        path_positions += [(start.get_x(), start.get_y() - i) for i in range(0, abs(distance_y), 2)]

            path_positions.append((0, 0))

            all_path_positions.append(path_positions)

        smallest_path_positions = min(all_path_positions, key=len)
        
        with open(self.fname + ".path", "w") as file:
            for position in smallest_path_positions:
                print(f'{position[0]} {position[1]}', file=file)

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
