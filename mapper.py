from typing import List, Iterable

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
