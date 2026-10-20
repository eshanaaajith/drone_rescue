# Disaster Rescue Drone Swarm Simulation

An autonomous multi-agent drone swarm simulation for disaster search and rescue operations, built with Python and Pygame. The system demonstrates coordinated spatial exploration, real-time dynamic obstacle avoidance, frontier exploration, and optimal path planning using the A* algorithm.

---

## Table of Contents

- [Overview](#overview)
- [Key Features](#key-features)
- [System Architecture](#system-architecture)
  - [1. Search Drones (Exploration)](#1-search-drones-exploration)
  - [2. Response Drones (Rescue & Delivery)](#2-response-drones-rescue--delivery)
  - [3. Dynamic Environment](#3-dynamic-environment)
- [Project Structure](#project-structure)
- [Installation & Setup](#installation--setup)
- [Usage Guide](#usage-guide)
  - [Running the Graphical Simulation](#running-the-graphical-simulation)
  - [Running Unit Tests](#running-unit-tests)
  - [Running Monte Carlo Benchmarks](#running-monte-carlo-benchmarks)
- [Algorithms & Metrics](#algorithms--metrics)
- [Requirements](#requirements)
- [License](#license)

---

## Overview

In post-disaster scenarios (earthquakes, floods, structural collapses), rapid localization and supply delivery to victims are critical. This project models a collaborative multi-agent robotic system comprising:
- **Search Drones (Alpha, Beta, Gamma)**: Zone-partitioned frontier exploration agents that map unknown sectors and locate trapped victims.
- **Response Drones (Responders 1 & 2)**: Heavy-payload medical delivery agents stationed at the base that calculate optimal routes, deliver first aid, and re-plan trajectories in real time when dynamic hazards shift.

---

## Key Features

- **Decentralized Sector Partitioning**: Grid divided into dedicated search zones (`Zone 1`, `Zone 2`, `Zone 3`) to eliminate search overlap.
- **Frontier-Based Exploration**: Search drones dynamically identify open perimeter cells and navigate toward unexplored frontiers.
- **A* Pathfinding Algorithm**: Guarantees optimal shortest paths utilizing Manhattan distance heuristics.
- **Dynamic Hazard Replanning**: Obstacles (fires/debris) shift randomly during runtime. Drones detect blocked routes and trigger dynamic real-time re-routing.
- **Live Pygame Dashboard**: Real-time HUD displaying mission statistics, search drone states, response drone FSM states, collision counter, replan count, and rolling event logs.
- **Monte Carlo Benchmarking Suite**: Headless testing framework for evaluating discovery latency, mission turnaround, collision safety, and completion rates.

---

## System Architecture

```
                          +-------------------+
                          |    Base Station   |
                          |  (Spawn & Depot)  |
                          +---------+---------+
                                    |
          +-------------------------+-------------------------+
          |                                                   |
+---------v---------+                               +---------v---------+
|   Search Drones   | (Alpha, Beta, Gamma)          |  Response Drones  | (Resp 1, Resp 2)
| - Frontier Search |                               | - A* Pathfinding  |
| - Sector Bounded  |                               | - Dynamic Replan  |
| - Victim Discovery|                               | - Medical Delivery|
+---------+---------+                               +---------+---------+
          |                                                   ^
          +================ Victim Discovered ===============+
```

### 1. Search Drones (Exploration)
- Assigned to distinct spatial bounds across the grid (`ZONE_1_BOUNDS`, `ZONE_2_BOUNDS`, `ZONE_3_BOUNDS`).
- Maintain an internal set of visited coordinates.
- Calculate candidate frontier tiles adjacent to visited cells and navigate via local A* pathfinding.
- Transition to `SECTOR DONE` once all reachable frontiers within their bounds are covered.

### 2. Response Drones (Rescue & Delivery)
- Finite State Machine (FSM): `IDLE` -> `RESPONDING` -> `RETURNING` -> `IDLE`.
- Prioritize unserviced victims using a **nearest-victim-first** policy.
- Continuously check the next step along their planned route; if a moving hazard blocks the path, they trigger an immediate A* re-plan or wait safely.

### 3. Dynamic Environment
- **Grid Size**: 25x15 cells.
- **Hazards (Red)**: Obstacles with dynamic drift probability.
- **Victims (Blinking Red Crosses)**: Trapped survivors requiring emergency relief.
- **Base Station (Green 'B')**: Starting and replenishment hub for response drones.

---

## Project Structure

```
.
├── drone_rescue_simulation.py   # Main Pygame visual simulation & drone swarm engine
├── benchmark.py                 # Headless Monte Carlo benchmark tool
├── test_astar.py                # Unit test suite verifying A* algorithm correctness
├── requirements.txt             # Python dependencies
└── README.md                    # Project documentation
```

---

## Installation & Setup

### Prerequisites
- Python 3.8 or higher
- `pip` package manager

### 1. Clone or Download the Repository
```bash
git clone https://github.com/your-username/drone-rescue-simulation.git
cd drone-rescue-simulation
```

### 2. Create and Activate a Virtual Environment (Recommended)

**On Windows (PowerShell):**
```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
```

**On Linux / macOS:**
```bash
python3 -m venv venv
source venv/bin/activate
```

### 3. Install Dependencies
```bash
pip install -r requirements.txt
```

---

## Usage Guide

### Running the Graphical Simulation

Launch the interactive Pygame window:
```bash
python drone_rescue_simulation.py
```

To run with a reproducible random seed for demonstrations or vivas:
```bash
python drone_rescue_simulation.py 42
```

**Controls:**
- <kbd>Space</kbd>: Pause / Resume simulation.
- <kbd>Close Window</kbd>: Exit simulation.

---

### Running Unit Tests

Execute the unit test suite validating path optimality, wall detours, and boxed-in obstacle handling:
```bash
python test_astar.py
```

Expected output:
```text
PASS test_extra_start_equals_goal
PASS test_u01_optimal_on_empty_grid
PASS test_u02_detours_around_wall
PASS test_u03_unreachable_goal_returns_none
4/4 passed
```

---

### Running Monte Carlo Benchmarks

Evaluate the robustness and multi-agent coordination performance across multiple randomized runs (headless, without opening a GUI window):

```bash
# Run standard benchmark (30 iterations)
python benchmark.py

# Run customized benchmark (100 runs, custom step cap)
python benchmark.py --runs 100 --cap 1500 --seed0 10
```

**Output Metrics:**
- **Success Rate**: Percentage of runs where all victims were rescued within the step limit.
- **Mission Completion Time**: Total steps taken until all victims are rescued and responders return to base.
- **Victim Discovery Latency**: Mean step at which victims are first spotted by search drones.
- **Rescue Turnaround**: Mean steps elapsed between victim discovery and medicine delivery.
- **Hazard Collisions**: Number of steps any drone spent inside hazard cells (designed to remain 0).

---

## Algorithms & Metrics

| Component | Technique / Metric | Description |
|---|---|---|
| **Pathfinding** | $A^*$ Search Algorithm | Employs Manhattan Distance heuristic $h(x, y) = \|x_1 - x_2\| + \|y_1 - y_2\|$ for admissible, optimal grid pathfinding. |
| **Search Pattern** | Frontier Exploration | Search drones expand boundaries outward from visited nodes to unvisited non-hazard tiles. |
| **Collision Avoidance** | Pre-step Route Validation | Real-time sensor check prior to movement; drops blocked cached paths and triggers dynamic re-planning. |
| **Dispatch Policy** | Nearest-Victim Greedy Selection | Sorts detected victims by heuristic distance to idle responder; skips unreachable targets to avoid deadlock. |

---

## Requirements

The project relies on standard, lightweight Python libraries:
- `pygame >= 2.1.0`
- `numpy >= 1.20.0`

*(See [requirements.txt](file:///c:/Users/eshan/Downloads/Foai%20%282%29/Foai/requirements.txt) for exact package specifications).*
