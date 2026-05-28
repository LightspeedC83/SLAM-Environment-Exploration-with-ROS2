# Environment Mapper — Autonomous SLAM Robot

A fully autonomous robot that explores and maps unknown environments using **Simultaneous Localization and Mapping (SLAM)**, built with Python, ROS2, and the Stage simulator.

[Watch Demo — Probabilistic Mapping](https://youtu.be/Ud5Zz3YyYBg)

[![Probabilistic Mapping Demo](https://img.youtube.com/vi/Ud5Zz3YyYBg/0.jpg)](https://youtu.be/Ud5Zz3YyYBg)

---

## Overview

This project implements a frontier-based exploration robot that builds a map of its environment as it moves through it — without any prior knowledge of the space. The robot continuously identifies unexplored regions (frontiers), plans paths toward them, and updates its internal map using laser sensor data.

**Core capabilities:**
- Real-time 2D occupancy grid mapping
- Frontier-based autonomous exploration
- Obstacle inflation and collision recovery
- Dijkstra pathfinding with cost-weighted obstacle avoidance
- Probabilistic (log-odds) mapping 

---

## Demo

Robot in the Stage simulator alongside the SLAM map visualized in rviz2:

![Robot and map visualized](assets/robot%20and%20map%20vizualized%20(1).png)

Gaussian-smoothed map used as input to the frontier-finding BFS algorithm, and the resulting path to the nearest frontier cell:

![Visualized SLAM map](assets/vizualized%20slam%20map%20(2).png)

Sequential frontier exploration paths with obstacle inflation as the robot discovers the environment:

![Frontier path sequence](assets/vizualized%20slam%20map%20obstacle%20inflation%20and%20path%20sequence%20(3).png)

---

## How It Works

### Mapping
The robot maintains a 2D occupancy grid where each cell is one of:
- `0` — free space
- `100` — occupied
- `-1` — unexplored

The map initializes as a square array scaled by `SLAM_MAP_RESOLUTION_SCALAR × max_sensor_range × 2`, and **dynamically doubles in size** whenever the robot encounters points outside the current map bounds.

Laser scan data is processed using the **Bresenham line algorithm** (all-octants version) to trace rays through the grid and mark free/occupied cells. A downsampling parameter controls how frequently scans are processed for performance.

### Frontier Exploration
The robot identifies *frontier cells* — unexplored cells adjacent to known free space — using BFS on a Gaussian-smoothed version of the map. It then plans a path to the nearest frontier using **Dijkstra's algorithm**, with obstacle inflation and smoothing incorporated into the node cost to naturally steer the robot away from walls.

### Collision Recovery
When the robot strays within `PROTECTION_RADIUS` meters of an obstacle, it enters **recovery mode**:
1. Stops immediately
2. Waits `LASER_SLEEP_TIME_AFTER_INTERRUPT` seconds for new sensor data
3. Recalculates path to the nearest frontier using fresher map data

Traditional obstacle inflation (expanding occupied cells by the protection radius) is also applied before pathfinding.

### Probabilistic Mapping 
The standard binary map can shift slightly as new sensor data arrives. The probabilistic mode addresses this by using **log-odds updates** on cell occupancy probability, weighted by distance from the laser scanner — closer readings carry higher confidence. This produces a stable, continuous probability map that accumulates evidence over time.

![Probabilistic mapping](assets/probabalistic%20mapping%20(4).png)

---

## Tech Stack

- **Language:** Python
- **Framework:** ROS2
- **Simulator:** Stage
- **Visualization:** rviz2
- **Algorithms:** Bresenham line, Dijkstra, BFS, Gaussian smoothing, log-odds probabilistic mapping

---

## Configuration

All behavior is controlled via parameters at the top of the node file:

| Parameter | Default | Description |
|---|---|---|
| `LINEAR_VELOCITY` | `0.5` | Forward speed of the robot (m/s) |
| `ANGULAR_VELOCITY` | `0.2` | Turning speed of the robot (rad/s) |
| `SLAM_MAP_RESOLUTION_SCALAR` | `10` | Grid cells per meter |
| `PROBABILISTIC_MAPPING` | `True` | Enable log-odds probabilistic map |
| `CELL_PROBABILITY_OCCUPIED` | `70` | Probability threshold to mark a cell occupied |
| `SCAN_DOWNSAMPLING` | `1` | Process every nth laser scan (1 = all scans) |
| `SMOOTHING_KERNEL_SIZE` | `10` | Gaussian kernel size for frontier detection |
| `SMOOTHING_SIGMA` | `6` | Sigma for Gaussian smoothing |
| `MAP_CLEAR_THRESHOLD` | `20` | Occupancy value below which a cell is traversable |
| `PROTECTION_RADIUS` | `0.3` | Obstacle proximity threshold for recovery mode (m) |
| `LASER_SLEEP_TIME_AFTER_INTERRUPT` | `30` | Sleep duration (s) on recovery mode entry |

---

## Known Limitations & Future Work

- **Frontier ranking:** Currently the robot always targets the *nearest* frontier. Ranking frontiers by expected information gain could improve exploration efficiency.
- **Map smoothing cost:** Gaussian smoothing is recomputed on every frontier search cycle, which becomes expensive as the map grows.
- **Open-loop movement:** Navigation to waypoints is currently open-loop. A closed-loop controller tracking the planned path would be more robust and reduce reliance on the collision recovery fallback.

---

## Acknowledgements

All code is original work. Portions adapted from a prior assignment (PA3). Thanks to Dr. Quattrini Li for his help. 