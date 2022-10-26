class Intersection: 

    def __init__(self):

        self.possible_paths = set()
        self.visited_paths = set()
        self.neighbours = set()

    def __init__(self, x, y):

        self.possible_paths = set()
        self.visited_paths = set()
        self.neighbours = set()

        self.x = x
        self.y = y

    def add_path(self, path):
        self.possible_paths.add(path)

    def add_visited_path(self, path):
        self.visited_paths.add(path)

    def add_neighbour(self, neighbour):
        self.neighbours.add(neighbour)

    def get_possible_paths(self):
        return self.possible_paths

    def get_visited_paths(self):
        return self.visited_paths

    def get_neighbours(self):
        return self.neighbours

    def get_x(self):
        return self.x

    def get_y(self):
        return self.y

    def get_coordinates(self):
        return (self.x, self.y)

    def __str__(self): return str(self.get_coordinates())
    def __repr__(self): return str(self)