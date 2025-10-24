# Sphero Simulation

A ROS 2 simulation package for Sphero robots using the Stage simulator.

## Contents

- **sphero_description**: URDF models and robot description files for Sphero robots
- **sphero_stage**: Stage simulation environment with configurable robot spawning and map support

## Features

- Configurable number of robots (1-N)
- Multiple robot formation patterns (circle, line)
- Pre-built maps (empty, mazes, random obstacles)
- RViz2 visualization support
- ROS 2 Navigation stack integration

## Usage

1. **Build the packages:**
   ```bash
   colcon build --packages-select sphero_description sphero_stage
   source install/setup.bash
   ```

2. **Launch simulation:**
   ```bash
   # Basic simulation
   ros2 launch sphero_stage start_ros2.py

   # With RViz visualization
   ros2 launch sphero_stage start_ros2.py start_rviz:=true
   ```

3. **Configure simulation:**
   Edit `sphero_stage/launch/launch_params.yaml` to modify:
   - Number of robots
   - Map selection
   - Robot formation patterns

For detailed usage and configuration options, see the README files in the individual package directories.