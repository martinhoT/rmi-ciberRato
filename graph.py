from typing import Set, Tuple
from directions import Direction


class Node:

    def __init__(self, x: int, y: int):
        self.neighbours: Set[Node] = set()

        self.x: int = x
        self.y: int = y

    def add_neighbour(self, neighbour: 'Node'):
        self.neighbours.add(neighbour)

    def get_neighbours(self) -> Set['Node']:
        return self.neighbours

    def get_x(self) -> int:
        return self.x

    def get_y(self) -> int:
        return self.y

    def get_coordinates(self) -> Tuple[int, int]:
        return (self.x, self.y)

    def __str__(self): return self.__class__.__name__ + str(self.get_coordinates())
    def __repr__(self): return str(self)


class Intersection(Node):

    def __init__(self, x: int, y: int):
        super().__init__(x, y)
        self.possible_paths: Set[Direction] = set()
        self.visited_paths: Set[Direction] = set()

    def add_path(self, path: Direction):
        self.possible_paths.add(path)

    def add_visited_path(self, path: Direction):
        self.visited_paths.add(path)

    def get_possible_paths(self) -> Set[Direction]:
        return self.possible_paths

    def get_visited_paths(self) -> Set[Direction]:
        return self.visited_paths


class Checkpoint(Node):

    def __init__(self, x: int, y: int, identifier: int):
        super().__init__(x, y)
        self.id: int = identifier
    
    def get_identifier(self) -> int:
        return self.id

    def __str__(self): return super().__str__() + '-' + str(self.id)
