from typing import Callable, Dict, List, Tuple
from directions import Direction
from graph import Intersection, Node

def map_to_text(positions: List[Tuple[int, int]]) -> List[str]:
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

    return grid

def wavefront_expansion(start_node: Node, key: Callable[[Node], bool], max_distance: int=None) -> List[Node]:
    """Wavefront expansion algorithm to find the path to the closest node that satisfies the `key` condition."""

    # (Node, Path excluding this node, Path distance)
    nodes_to_explore  = [(start_node, [], 0)]
    neighbours = None
    checked_nodes = []
    distance_to_this_point = lambda t: t[2]

    while nodes_to_explore:

        nodes_to_explore.sort(key=distance_to_this_point, reverse=True)
        this_node, previous_nodes, previous_distance = nodes_to_explore.pop()
        neighbours = this_node.get_neighbours()

        if max_distance and previous_distance > max_distance:
            return None

        checked_nodes.append(this_node)

        for neighbour in neighbours:

            if key(neighbour):
                return previous_nodes + [this_node, neighbour]

            if neighbour not in checked_nodes:
                distance_x = abs(this_node.get_x() - neighbour.get_x())
                distance_y = abs(this_node.get_y() - neighbour.get_y())
                distance = distance_x + distance_y

                nodes_to_explore.append((neighbour, previous_nodes + [this_node], previous_distance + distance))
    
    return None


class Navigator:
    """Utility functions used for mapping and navigation involving positions and orientations."""

    # Obtain orientation of robot in the map
    @classmethod
    def get_direction(cls, compass: float) -> Direction:
        if compass >= 45 and compass <= 135:
            return Direction.N
        elif compass < 45 and compass > -45:
            return Direction.E
        elif compass <= -45 and compass >= -135:
            return Direction.S
        else:
            return Direction.W
    
    @classmethod
    def round_pos(cls, x: float, y: float, starting_position: Tuple[float, float]) -> Tuple[int, int]:
        # Assumed that the robot is still in the starting position and hasn't updated it
        if not starting_position:
            return 0, 0
        return round(x-starting_position[0]), round(y-starting_position[1])

    @classmethod
    def round_pos_to_intersection(cls, x: float, y: float, starting_position: Tuple[float, float]) -> Tuple[int, int]:
        if not starting_position:
            return 0, 0
        return round((x-starting_position[0])/2)*2, round((y-starting_position[1])/2)*2

    @classmethod
    def get_direction_from_path(cls, start: Node, end: Node) -> Direction:

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

    @classmethod
    def get_distance_to_closest_intersection_in_front_of_pos(cls,
            position: Tuple[int, int],
            direction: Direction,
            intersections: Dict[Tuple[int, int], Intersection]) -> int:
        
        # If the intersection in front of me is far away, then speed up
        intersection_in_front_distance = {
            Direction.E: lambda i, p: i[0] - p[0] if i[1] == p[1] else -1,
            Direction.W: lambda i, p: p[0] - i[0] if i[1] == p[1] else -1,
            Direction.N: lambda i, p: i[1] - p[1] if i[0] == p[0] else -1,
            Direction.S: lambda i, p: p[1] - i[1] if i[0] == p[0] else -1
        }[direction]

        distance_of_intersections_in_front_of_me = [intersection_in_front_distance(intersection, position) for intersection in intersections
            if intersection_in_front_distance(intersection, position) > 0]

        if distance_of_intersections_in_front_of_me:
            return min(distance_of_intersections_in_front_of_me)
        
        return None

    @classmethod
    def get_walkable_distance_to_closest_intersection_in_front_of_pos(cls,
            position: Tuple[int, int],
            direction: Direction,
            intersections: Dict[Tuple[int, int], Intersection],
            pmap: List[Tuple[int, int]]) -> int:

        closest_distance = cls.get_distance_to_closest_intersection_in_front_of_pos(
            position, direction, intersections)
        
        if closest_distance:

            intersection_step = {
                Direction.E: lambda i, n: (i[0] + n, i[1]),
                Direction.W: lambda i, n: (i[0] - n, i[1]),
                Direction.N: lambda i, n: (i[0], i[1] + n),
                Direction.S: lambda i, n: (i[0], i[1] - n)
            }[direction]

            positions_to_be_covered = {intersection_step(position, n) for n in range(1, closest_distance)}
            
            # Should reach that intersection in a known straight path from this position
            if len(positions_to_be_covered - set(pmap)) == 0:
                return closest_distance
        
        return None
    
    @classmethod
    def get_angle_to_track(cls, compass: float):
        direction = cls.get_direction(compass)
        return compass - \
                 (90 if direction == Direction.N
            else -90 if direction == Direction.S
            else  180 if compass > 135
            else -180 if compass < -135
            else 0)
