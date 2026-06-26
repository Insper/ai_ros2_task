# File Guide

This guide summarizes the main files in the `robot_ai` package.

## `robot_ai/robot_ai/astar.py`

Student-facing planner template. It defines `AStarPathfinder`, the class used by
the navigation wrapper. The constructor, plotting helper, map-loading helper,
and pipeline method are provided. Students complete the search-related methods.

## `robot_ai/robot_ai/astar_gab.py`

Instructor reference implementation of the same interface defined in
`astar.py`. It is useful for validating the expected behavior of the planner
pipeline, potential field, unknown-space filtering, and waypoint simplification.

## `robot_ai/robot_ai/map.py`

ROS 2 helper that subscribes to `/odom`, `/tf`, and `/map`, converts ROS
occupancy-grid data into the planner's grayscale representation, tracks the
robot pose in the map frame, and converts planned map cells back into world
coordinates.

## `robot_ai/robot_ai/navigator.py`

ROS 2 node that combines `MapProcessor` and Nav2's `NavigateToPose` action. It
requests a path, sends each waypoint as a Nav2 goal, waits for completion, and
replans when the current known prefix has been exhausted.

## `robot_ai/robot_ai/amcl.py`

Pose and map helper used by the direct-control example. It subscribes to odometry,
transforms, and map data, then exposes the robot pose and map conversion methods.

## `robot_ai/robot_ai/goto.py`

Example proportional controller that drives directly toward a single point using
`cmd_vel`. The main navigation activity uses Nav2 through `navigator.py`; this
file is kept as a lower-level control example.

## `robot_ai/robot_ai/base_controller.py`

Minimal waypoint-navigation example that sends one fixed `NavigateToPose` goal.
It is useful as a compact reference for Nav2 action-client usage.

## `robot_ai/robot_ai/examples/`

PGM maps used to test the planner without ROS. They encode free, occupied, and
unknown cells in the same format described in the paper draft.
