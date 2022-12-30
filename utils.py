import math
from typing import Callable, Dict, Iterable, List, Tuple, Union
from directions import Direction
from graph import Node
from robData import MovementData, RobData


CENTER_TO_LINE_SENSOR = 0.438

def map_to_text(positions: List[Tuple[int, int]],
        checkpoints: Dict[int, Tuple[int, int]]=None,
        intersections: List[Tuple[int, int]]=None) -> List[str]:
    start_position = (0, 0)
    gwidth = 49
    gheight = 21

    gcenter = (gwidth//2, gheight//2)

    grid = [[' ']*gwidth for _ in range(gheight)]

    sub = lambda p1, p2: (p1[0]-p2[0], p1[1]-p2[1])
    add = lambda p1, p2: (p1[0]+p2[0], p1[1]+p2[1])

    for position in positions:
        offset = sub(position, start_position)
        grid_pos = add(gcenter, offset)
        grid[-grid_pos[1] - 1][grid_pos[0]] = \
                '-' if offset[0] % 2 == 1 and offset[1] % 2 == 0 else \
                '|' if offset[0] % 2 == 0 and offset[1] % 2 == 1 else \
                ' '
    
    grid[gcenter[1]][gcenter[0]] = 'I'

    if intersections is not None:
        for intersection in intersections:
            offset = sub(intersection, start_position)
            grid_pos = add(gcenter, offset)
            grid[-grid_pos[1] - 1][grid_pos[0]] = '*'

    if checkpoints is not None:
        for checkpoint_id, checkpoint_pos in checkpoints.items():
            offset = sub(checkpoint_pos, start_position)
            grid_pos = add(gcenter, offset)
            grid[-grid_pos[1] - 1][grid_pos[0]] = str(checkpoint_id)

    return grid


def manhattan_distance(p1: Tuple[int, int], p2: Tuple[int, int]) -> int:
    distance_x = abs(p1[0] - p2[0])
    distance_y = abs(p1[1] - p2[1])
    return distance_x + distance_y


def wavefront_expansion(start_node: Node, key: Callable[[Node], bool], max_distance: int=None) -> Tuple[List[Node], int]:
    """Wavefront expansion algorithm to find the path to the closest node that satisfies the `key` condition."""

    # (Node, Path excluding this node, Path distance)
    nodes_to_explore  = [(start_node, [], 0)]
    neighbours = None
    checked_nodes = []
    distance_so_far = lambda t: t[2]

    while nodes_to_explore:

        nodes_to_explore.sort(key=distance_so_far, reverse=True)
        this_node, previous_nodes, previous_distance = nodes_to_explore.pop()
        neighbours = this_node.get_neighbours()

        if max_distance and previous_distance > max_distance:
            return None

        checked_nodes.append(this_node)

        neighbours_distances = sorted(( (neighbour, manhattan_distance(this_node.get_coordinates(), neighbour.get_coordinates())) for neighbour in neighbours ), key=lambda t: t[1])
        for neighbour, distance in neighbours_distances:

            if key(neighbour):
                return previous_nodes + [this_node, neighbour], previous_distance + distance

            if neighbour not in checked_nodes:
                nodes_to_explore.append((neighbour, previous_nodes + [this_node], previous_distance + distance))
    
    return None


# Obtain orientation of robot in the map
def get_direction(compass: float) -> Direction:
    if compass >= 45 and compass <= 135:
        return Direction.N
    elif compass < 45 and compass > -45:
        return Direction.E
    elif compass <= -45 and compass >= -135:
        return Direction.S
    else:
        return Direction.W


def round_pos(x: float, y: float, starting_position: Tuple[float, float]) -> Tuple[int, int]:
    # Assumed that the robot is still in the starting position and hasn't updated it
    if not starting_position:
        return 0, 0
    return round(x-starting_position[0]), round(y-starting_position[1])


def round_pos_to_intersection(x: float, y: float, starting_position: Tuple[float, float], direction: Direction) -> Tuple[int, int]:
    if not starting_position:
        return 0, 0
    if direction:
        x, y = walk_in_direction((x, y), direction, CENTER_TO_LINE_SENSOR)   # align robot's center to the intersection
    return round((x-starting_position[0])/2)*2, round((y-starting_position[1])/2)*2


def get_direction_from_path(start: Node, end: Node) -> Direction:

    # Same x
    if start.get_x() == end.get_x():
        if end.get_y() > start.get_y():
            return Direction.N
        else:
            return Direction.S
    
    # Same y
    else:
        if end.get_x() > start.get_x():
            return Direction.E
        else:
            return Direction.W


def walk_in_direction(position: Tuple[float, float], 
        direction: Direction,
        steps: float) -> Tuple[float, float]:

    walk = {
        Direction.E: lambda p, s: (p[0] + s, p[1]),
        Direction.W: lambda p, s: (p[0] - s, p[1]),
        Direction.N: lambda p, s: (p[0], p[1] + s),
        Direction.S: lambda p, s: (p[0], p[1] - s),
    }[direction]
    
    return walk(position, steps)


def get_distance_to_closest_intersection_in_front_of_pos(
        position: Tuple[float, float],
        direction: Direction,
        intersections: Iterable[Tuple[int, int]],
        rounded_position: Tuple[int, int]=None) -> Tuple[float, Tuple[int, int]]:

    # The position is already rounded
    if not rounded_position:
        rounded_position = position

    p = position
    r = rounded_position

    # If the intersection in front of me is far away, then speed up
    intersection_in_front_distance = {
        Direction.E: lambda i: i[0] - p[0] if i[1] == r[1] else -1,
        Direction.W: lambda i: p[0] - i[0] if i[1] == r[1] else -1,
        Direction.N: lambda i: i[1] - p[1] if i[0] == r[0] else -1,
        Direction.S: lambda i: p[1] - i[1] if i[0] == r[0] else -1
    }[direction]

    distance_of_intersections_in_front_of_me = [
        (distance, intersection) 
        for distance, intersection 
        in zip(map(intersection_in_front_distance, intersections), intersections)
        if distance > 0
    ]

    return min(distance_of_intersections_in_front_of_me, default=None)


def get_walkable_distance_to_closest_intersection_in_front_of_pos(
        position: Tuple[float, float],
        direction: Direction,
        intersections: Iterable[Tuple[int, int]],
        pmap: List[Tuple[int, int]],
        rounded_position: Tuple[int, int]=None) -> Tuple[float, Tuple[int, int]]:

    if not rounded_position:
        rounded_position = position

    closest = get_distance_to_closest_intersection_in_front_of_pos(
        position, direction, intersections, rounded_position)
    
    if closest:
        closest_distance, closest_intersection = closest

        intersection_step = {
            Direction.E: lambda i, n: (i[0] + n, i[1]),
            Direction.W: lambda i, n: (i[0] - n, i[1]),
            Direction.N: lambda i, n: (i[0], i[1] + n),
            Direction.S: lambda i, n: (i[0], i[1] - n)
        }[direction]

        positions_to_be_covered = {intersection_step(rounded_position, n) for n in range(1, math.ceil(closest_distance))}
        
        # Should reach that intersection in a known straight path from this position
        if len(positions_to_be_covered - set(pmap)) == 0:
            return closest_distance, closest_intersection
    
    return None


def get_angle_to_track(compass: float):
    direction = get_direction(compass)
    return compass - \
                (90 if direction == Direction.N
        else -90 if direction == Direction.S
        else  180 if compass > 135
        else -180 if compass < -135
        else 0)


def estimate_pos(x: float, y: float, compass: float, starting_position: Tuple[float, float]) -> Tuple[Union[int, float], Union[int, float]]:

    direction = get_direction(compass)

    # If the robot is facing north or south, x is rounded to the nearest even number
    if direction == Direction.N or direction == Direction.S:
        return round((x-starting_position[0])/2)*2, y

    # If the robot is facing east or west, y is rounded to the nearest even number
    elif direction == Direction.E or direction == Direction.W:
        return x, round((y-starting_position[1])/2)*2

    

def calculate_next_movement(
    current_in: Tuple[float, float], 
    movement: MovementData) -> MovementData:

    current_in_left = max(-0.15, min(current_in[0], 0.15))
    current_in_right = max(-0.15, min(current_in[1], 0.15))

    previous_out_left = movement.out[0]
    previous_out_right = movement.out[1]
    
    current_out_left = (current_in_left + previous_out_left) / 2
    current_out_right = (current_in_right + previous_out_right) / 2

    lin = (current_out_left + current_out_right)/2

    previous_x = movement.coordinates[0]
    previous_y = movement.coordinates[1]

    # Calculate next (x, y) position
    x = previous_x + lin * math.cos(movement.angle)
    y = previous_y + lin * math.sin(movement.angle)

    D = 1 # Robot diameter
    rot = (current_out_right - current_out_left)/D

    # Calculate next angle
    angle = movement.angle + rot

    return MovementData((current_out_left, current_out_right), (x, y), angle)


def update_checkpoints_neighbours(rdata: RobData):

    # Obtain checkpoints neighbours
    checkpoints = list(rdata.checkpoints.values())
    for checkpoint in checkpoints:

        coordinates = checkpoint.get_coordinates()
        if coordinates in rdata.intersections:
            intersection = rdata.intersections[coordinates]
            neighbours = intersection.get_neighbours()
            for neighbour in neighbours:
                checkpoint.add_neighbour(neighbour)
                neighbour.add_neighbour(checkpoint)

        else:
            
            x = coordinates[0]
            y = coordinates[1]

            # Check if 0y
            if (x, y-1) in rdata.pmap or (x, y+1) in rdata.pmap:

                intersections_at_y = [i for i in rdata.intersections.values() if i.get_x() == x]

                closest_intersection_at_down = min((i for i in intersections_at_y if i.get_y() < y), key=lambda intersection: abs(intersection.get_y() - y), default=None)
                closest_intersection_at_up = min((i for i in intersections_at_y if i.get_y() > y), key=lambda intersection: abs(intersection.get_y() - y), default=None)
                
                # Intersection has to be walkable
                closest_intersection_at_down = None if (x, y-1) not in rdata.pmap else closest_intersection_at_down
                closest_intersection_at_up = None if (x, y+1) not in rdata.pmap else closest_intersection_at_up
                
                if closest_intersection_at_down:
                    checkpoint.add_neighbour(closest_intersection_at_down)
                    closest_intersection_at_down.add_neighbour(checkpoint)

                if closest_intersection_at_up:
                    checkpoint.add_neighbour(closest_intersection_at_up)
                    closest_intersection_at_up.add_neighbour(checkpoint)


            # Check if 0x
            elif (x-1, y) in rdata.pmap or (x+1, y) in rdata.pmap:
                
                intersections_at_x = [i for i in rdata.intersections.values() if i.get_y() == y]

                closest_intersection_at_left = min((i for i in intersections_at_x if i.get_x() < x), key=lambda intersection: abs(intersection.get_x() - x), default=None)
                closest_intersection_at_right = min((i for i in intersections_at_x if i.get_x() > x), key=lambda intersection: abs(intersection.get_x() - x), default=None)
                
                # Intersection has to be walkable
                closest_intersection_at_left = None if (x-1, y) not in rdata.pmap else closest_intersection_at_left
                closest_intersection_at_right = None if (x+1, y) not in rdata.pmap else closest_intersection_at_right
                
                if closest_intersection_at_left:
                    checkpoint.add_neighbour(closest_intersection_at_left)
                    closest_intersection_at_left.add_neighbour(checkpoint)

                if closest_intersection_at_right:
                    checkpoint.add_neighbour(closest_intersection_at_right)
                    closest_intersection_at_right.add_neighbour(checkpoint)