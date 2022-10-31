import itertools
import sys
from croblink import *
from math import *
import xml.etree.ElementTree as ET
from directions import opposite_direction

from graph import Checkpoint, Intersection
from intention import Rotate, Finish
from utils import get_direction, manhattan_distance, map_to_text, wavefront_expansion
from robData import RobData

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
    def __init__(self, robName, rob_id, angles, host, fname='robC2'):
        CRobLinkAngs.__init__(self, robName, rob_id, angles, host)
        
        # The map has been sufficiently traversed, no need to map the rest of the intersections
        def sufficient_map(rdata: RobData) -> bool:
            if len(rdata.intersections) == 0 \
                    or len(rdata.checkpoints) != int(self.nBeacons):
                return False

            # For every unexplored intersection, check if it's worth it to explore it
            # If not, consider the non-visited paths as being already visited
            unexplored_intersections = [i for i in rdata.intersections.values() if len(i.get_possible_paths() - i.get_visited_paths()) > 0]
            for unexplored_intersection in unexplored_intersections:
                # Explore if manhattan distance to any other unexplored intersection is
                # less than the actual distance to those intersections
                worth_exploring = False
                for other_unexplored_intersection in unexplored_intersections:
                    if other_unexplored_intersection != unexplored_intersection:
                        minimum_distance = manhattan_distance(unexplored_intersection.get_coordinates(), other_unexplored_intersection.get_coordinates())
                        known_path = wavefront_expansion(
                            unexplored_intersection,
                            key=lambda n: isinstance(n, Intersection) and n == other_unexplored_intersection,
                            max_distance=minimum_distance)
                        
                        if known_path is None:
                            worth_exploring = True
                            break

                if not worth_exploring:
                    # Consider the unexplored intersection has having already been explored
                    unexplored_intersection.possible_paths = unexplored_intersection.get_visited_paths()

            return all(len(i.get_possible_paths() - i.get_visited_paths()) == 0 for i in rdata.intersections.values())

        self.data = RobData(
            finish_condition=sufficient_map
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

        # Obtain checkpoints neighbours
        checkpoints = list(self.data.checkpoints.values())
        for checkpoint in checkpoints:

            coordinates = checkpoint.get_coordinates()
            if coordinates in self.data.intersections:
                intersection = self.data.intersections[coordinates]
                neighbours = intersection.get_neighbours()
                for neighbour in neighbours:
                    checkpoint.add_neighbour(neighbour)
                    neighbour.add_neighbour(checkpoint)

            else:
                
                x = coordinates[0]
                y = coordinates[1]

                # Check if 0y
                if (x, y-1) in self.data.pmap or (x, y+1) in self.data.pmap:

                    intersections_at_y = [i for i in self.data.intersections.values() if i.get_x() == x]

                    closest_intersection_at_down = min((i for i in intersections_at_y if i.get_y() < y), key=lambda intersection: abs(intersection.get_y() - y), default=None)
                    closest_intersection_at_up = min((i for i in intersections_at_y if i.get_y() > y), key=lambda intersection: abs(intersection.get_y() - y), default=None)
                    
                    # Intersection has to be walkable
                    closest_intersection_at_down = None if (x, y-1) not in self.data.pmap else closest_intersection_at_down
                    closest_intersection_at_up = None if (x, y+1) not in self.data.pmap else closest_intersection_at_up
                    
                    if closest_intersection_at_down:
                        checkpoint.add_neighbour(closest_intersection_at_down)
                        closest_intersection_at_down.add_neighbour(checkpoint)

                    if closest_intersection_at_up:
                        checkpoint.add_neighbour(closest_intersection_at_up)
                        closest_intersection_at_up.add_neighbour(checkpoint)


                # Check if 0x
                elif (x-1, y) in self.data.pmap or (x+1, y) in self.data.pmap:
                    
                    intersections_at_x = [i for i in self.data.intersections.values() if i.get_y() == y]

                    closest_intersection_at_left = min((i for i in intersections_at_x if i.get_x() < x), key=lambda intersection: abs(intersection.get_x() - x), default=None)
                    closest_intersection_at_right = min((i for i in intersections_at_x if i.get_x() > x), key=lambda intersection: abs(intersection.get_x() - x), default=None)
                    
                    # Intersection has to be walkable
                    closest_intersection_at_left = None if (x-1, y) not in self.data.pmap else closest_intersection_at_left
                    closest_intersection_at_right = None if (x+1, y) not in self.data.pmap else closest_intersection_at_right
                    
                    if closest_intersection_at_left:
                        checkpoint.add_neighbour(closest_intersection_at_left)
                        closest_intersection_at_left.add_neighbour(checkpoint)

                    if closest_intersection_at_right:
                        checkpoint.add_neighbour(closest_intersection_at_right)
                        closest_intersection_at_right.add_neighbour(checkpoint)

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
