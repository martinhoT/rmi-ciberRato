from typing import Callable, List
from graph import Node

def map_to_text(positions: List[int]) -> List[str]:
    start_position = positions[0]
    gwidth = 49
    gheight = 21
    gcenter = (gwidth//2, gheight//2) 

    grid = [[' ']*gwidth for _ in range(gheight)]

    grid[gcenter[1]][gcenter[0]] = 'I'

    sub = lambda p1, p2: (p1[0]-p2[0], p1[1]-p2[1])
    add = lambda p1, p2: (p1[0]+p2[0], p1[1]+p2[1])

    for position in positions[1:]:
        offset = sub(position, start_position)
        grid_pos = add(gcenter, offset)
        grid[-grid_pos[1] - 1][grid_pos[0]] = \
                '-' if offset[0] % 2 == 1 and offset[1] % 2 == 0 else \
                '|' if offset[0] % 2 == 0 and offset[1] % 2 == 1 else \
                ' '

    return grid

def wavefront_expansion(start_node: Node, key: Callable[[Node], bool]) -> List[Node]:
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
