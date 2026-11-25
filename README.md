# AI ROS 2 Path Planning Activity

This repository contains the ROS 2 support package and A* path-planning
interface used in the educational activity described in the WRE 2026 paper
draft `WRE_2026.pdf`, **Teaching Artificial Intelligence through Physical Robot
Path Planning without Prior ROS Knowledge**.

The activity is designed for undergraduate Artificial Intelligence students.
Students implement a focused A* planner while the ROS 2 navigation stack,
mapping, localization, and robot-control details are handled by the instructor
infrastructure. This keeps the programming task centered on search and planning,
while still exposing the planner to realistic robotics constraints:

- partially observed occupancy-grid maps;
- obstacle clearance through a distance-transform potential field;
- replanning when unknown map regions become known;
- waypoint simplification for smoother physical robot motion.

## Repository Layout

```text
.
├── WRE_2026.pdf                  # Paper draft associated with this repository
├── README.md
└── robot_ai/
    ├── package.xml               # ROS 2 package metadata
    ├── setup.py                  # Python package entry points
    └── robot_ai/
        ├── astar.py              # Student-facing A* planner template
        ├── astar_gab.py          # Reference implementation used by instructors
        ├── map.py                # Occupancy-grid and coordinate conversion helper
        ├── navigator.py          # Nav2 NavigateToPose waypoint runner
        ├── amcl.py               # Pose/map helper used by direct control examples
        ├── goto.py               # Low-level point-to-point controller example
        └── examples/             # PGM maps for local planner tests
```

## System Context

The full classroom setup uses a TurtleBot3 Burger with ROS 2 Humble,
Cartographer, and Nav2. The robot publishes an occupancy grid on `/map`, odometry
on `/odom`, and transforms on `/tf`. The `robot_ai` package reads those topics,
builds the planner input, asks `AStarPathfinder` for waypoints, converts map
cells back into world coordinates, and sends each waypoint to Nav2 as a
`NavigateToPose` goal in the `map` frame.

Students do not need to edit ROS launch files or implement ROS nodes. Their main
task is to complete the methods in `robot_ai/robot_ai/astar.py`.

## Prerequisites

- ROS 2 Humble
- TurtleBot3 packages
- Nav2
- Cartographer for TurtleBot3
- Python 3
- Python packages used by the planner: `numpy`, `opencv-python`, `scipy`, and
  `matplotlib`

For TurtleBot3 simulations or physical robot runs, set the model before starting
the navigation stack:

```bash
export TURTLEBOT3_MODEL=burger
```

## Build

From a ROS 2 workspace that contains this repository:

```bash
colcon build --packages-select robot_ai
source install/setup.bash
```

If the package is cloned directly as a standalone repository, copy or symlink the
`robot_ai/` directory into the `src/` directory of a ROS 2 workspace before
building.

## Run With Online SLAM

Start Cartographer:

```bash
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```

Start Nav2 using the map topic produced by SLAM:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True \
  map:='topic://map'
```

Run the waypoint navigator:

```bash
ros2 run robot_ai navigator
```

## Run With a Saved Map

Save a map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/map
```

Start Nav2 with the saved map:

```bash
ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/map.yaml use_sim_time:=True
```

Then run:

```bash
ros2 run robot_ai navigator
```

## Standalone Planner Test

The A* planner can also be exercised without ROS by using the example PGM maps:

```bash
cd robot_ai/robot_ai/examples
python ../astar.py
```

The default `main()` in `astar.py` loads `map5.pgm`, builds an
`AStarPathfinder`, and plots the planned path when a valid implementation is
present.

## Planner Interface

Students implement these methods in `AStarPathfinder`:

- `preprocess_map()`: convert normalized occupancy values into a search map;
- `create_potential_field()`: penalize cells close to obstacles;
- `heuristic()`: compute the A* heuristic;
- `find_path()`: run A* over the occupancy grid;
- `reconstruct_path()`: recover the path from the predecessor map;
- `know_path()`: keep only the known and currently executable path prefix;
- `simplify_path()`: reduce the path into smoother waypoints.

The `run()` method orchestrates that pipeline and returns a simplified list of
map-cell coordinates to the ROS 2 navigation wrapper.

## Using This Project

Students must create a repository from the existing template at
https://github.com/Insper/ia_robotica.

## Citation

TODO

## License

This project is distributed under the Apache License 2.0. See
[`LICENSE`](LICENSE).
