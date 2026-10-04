# Project: Decentralized GNN Multi-Robot Grid Simulation

## Purpose

Build a cell-based warehouse simulation for training and evaluating a
decentralized Graph Neural Network (GNN) that coordinates multiple robots.
This is a new project copied from `multi_amr_ws`; the original Gazebo workspace
remains a separate project.

## Simulation model

- The warehouse is a static, discrete grid of cells.
- Walls and blocked cells do not move during a run.
- Robots occupy cells and move between neighboring free cells according to
  discrete actions (for example, stay, north, south, east, or west).
- Each robot has a start cell, a goal cell, and local observations of nearby
  cells and robots.
- Runs should support configurable grid dimensions, obstacles, robot count,
  start/goal positions, and episode length.
- Collision rules must prevent two robots from occupying the same cell or
  swapping through one another in the same simulation step.

## Multi-robot and GNN architecture

- Each robot is one graph node and has its own local agent/controller.
- A robot's node features include its grid position, goal, local occupancy,
  and task/progress information.
- Edges represent the robots whose messages are currently available to that
  robot. Begin with all-to-all communication; later allow range-limited links
  and communication failures.
- Each agent exchanges compact state/embedding messages with peers, builds its
  local graph view, runs the shared GNN policy, and selects its own next cell.
- No central runtime planner chooses actions for every robot. A simulator may
  enforce common physics and collision rules, while each agent computes its
  own action from its local state and received peer messages.
- Train the policy across episodes and report success rate, collisions,
  completion time, path length, and communication load.

## Separation from the copied workspace

The source packages and configuration were copied from `multi_amr_ws` into this
directory. Generated ROS `build/`, `install/`, and `log/` output was left out so
this workspace starts clean. The copied ROS/Gazebo packages are retained as
reference material while the grid simulator is developed. The inherited
`run_five_amr.sh` starts the Gazebo version and is not the runner for this
grid-only project; it will be replaced with a grid-simulation entry point as
implementation proceeds.

## Initial implementation direction

1. Add a standalone grid environment and deterministic episode runner.
2. Add one agent instance per robot and a message-passing interface between
   agents.
3. Add the GNN policy and training loop, with a simple baseline policy for
   comparison.
4. Add evaluation scenarios for congestion, blocked routes, and lost or
   delayed peer messages.
5. Keep ROS 2 as the inter-agent communication layer where useful, while
   removing Gazebo as a runtime dependency for this project.

## Workspace location

`/home/ksp/Documents/ChatGPT/ros2 amr/multi_amr_gnn_grid_ws`
