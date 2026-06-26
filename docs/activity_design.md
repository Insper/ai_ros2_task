# Activity Design

This repository supports an undergraduate Artificial Intelligence activity in
which students implement path planning for a physical mobile robot without
needing prior ROS 2 knowledge.

The classroom version uses an instructor-maintained robotics stack around a
TurtleBot3 Burger. ROS 2, Cartographer, Nav2, localization, transforms, and motor
commands are hidden behind a narrow Python interface. Students receive an
occupancy-grid map, a start cell, a goal cell, and two planner parameters. Their
implementation returns a list of map-cell waypoints.

## Learning Goals

- Apply A* to grid-based path planning.
- Use a heuristic suitable for spatial search.
- Account for obstacle clearance in the cost function.
- Handle unknown map regions under partial observability.
- Simplify waypoint paths so that the physical robot can move smoothly.
- Observe the difference between a path that works in a grid and a path that can
  be executed by a real robot.

## Map Representation

The planner receives a normalized grayscale occupancy map:

- `0`: obstacle;
- `128`: unknown or unexplored space;
- `255`: free space.

The original PGM maps use TurtleBot-style occupancy values. The helper functions
normalize those values before passing the map to the planner.

## Planning Pipeline

`AStarPathfinder.run()` executes the complete planning pipeline:

1. Search for a path using `find_path()`.
2. Recover the cell sequence with `reconstruct_path()`.
3. Remove unknown or currently unsafe segments with `know_path()`.
4. Reduce redundant waypoints with `simplify_path()`.
5. Optionally plot the full and simplified paths.

When the goal is beyond unknown space, `know_path()` returns only the known
prefix. The robot executes that prefix, observes more of the environment, and
the navigation wrapper asks the planner for a new path.

## Assessment Criteria

The physical robot evaluation emphasizes four behaviors:

- reaching the goal;
- keeping a safe distance from walls;
- producing smooth motion rather than abrupt turns;
- navigating narrow corridors and obstacles when feasible.

These criteria connect directly to the implemented methods: wall clearance comes
from the potential field, smoothness comes from waypoint simplification, and
replanning comes from unknown-space filtering.
