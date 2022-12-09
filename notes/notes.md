# Notes

## Useful information

### Robot proportions

```
Line sensor:
o _
o _|-> 0.08
o
o
o
o
o
```

- Line width: 0.2
- Distance to robot's center: 0.438
- Diameter: 1.0
- Cell width: 2.0
- Max speed: 0.15

### Tools

- We are free to use any standard libraries, including external ones as long as we include them in `build.sh`

### Troubleshooting

- In case there is a syntax error on `awk`, install `gawk`
- For maximum score on mapping, use the simulator's `mapping.out` for comparison instead of `robotmaze.out`

### Doubts

- How is the noise applied to the `lineSensor`? To a single bit? Is it a bit flip or only turn it to 1?
  - It's by bit flips, and an arbitrary number of them may happen in a single step (the noise applied to one bit is independent of the others)

## TO-DOs

### Problems

- `intention`: the distance to the known intersection ahead is not correct
- `robC4`: If the starting position is not an intersection, the `Finish` intention will not be triggered in the end

#### Issues to be aware of (but no need to fix)

- Imprecisions when the simulator is running too fast (10 cycles), either the program can't keep up or crucial steps are skipped.

### Optimizations

- `intention`: there is an inconsitency with which values of the `lineSensor` are evaluated in `CheckIntersectionForwardBacktrack`
- `intention`: the speed up function is not the best (speed up beyong `0.15` doesn't work in practice, heavily braking when not needed)
- `intention`: path calculation to finish may happen 1 intersection later
- Prioritize heading into directions that are closer to the map's borders
