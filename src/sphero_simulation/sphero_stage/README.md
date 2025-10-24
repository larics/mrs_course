# sphero_stage

Configuration and launch files for simulating Sphero robots (or any other simple mobile robot) in a simple 2D simulator Stage.

### Requirements

- ROS Humble & Python 3 (for ROS 1, see the `ros1` branch)
- Stage for ROS: [stage_ros2](https://github.com/tuw-robotics/stage_ros2)
- (_optional_) Keyboard teleoperation `sudo apt install ros-humble-teleop-twist-keyboard`

### Structure
Stage simulator requires three types of files:
- `map.bmp` - Bitmap file describing the map visuals
- `map.yaml`- Configuration file describing the resulution and origin of the map
- `map.world` - File describing the simulation world, including the used map, positions of robots, simulation parameters etc.

Map files are stored in `config/maps`. Since a new world file must be created every time the number or position of the robots changes, template world files are used to create actual files at runtime. Templates are stored in `config/world_templates` and created files in `config/worlds`.

**Adding new maps and world templates:** Create a bitmap representing the desired map and add corresponding .yaml and .world files using the provided ones as reference.

### Usage
1. Specify the map name, number of robots and their initial positions distribution within the map in `launch/launch_params.yaml`.
2. Run the `launch/start.launch.py` launch file. It will load the launch parameters, create the final world file and launch the simulator, map server, and the node for simulated TF.

### Step-by-step example
1. Leave all the values at their default settings.
1. Build your ROS workspace if you haven't done so: `cd <path_to_your_workspace> && colcon build --symlink-install`
1. In the 1st terminal: `ros2 launch sphero_stage start.launch.py` - A window with the simulator should open.
1. In the 2nd terminal: `ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=robot_0/cmd_vel` - Now you can control one of the robots by sending keystrokes to the 2nd terminal. Follow onscreen instructions. When done, you can exit with Ctrl-c.
1. In the 2nd terminal (after exiting previous command): `ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=robot_1/cmd_vel` - Now you can control the second robot same as the first one.
1. In the 3rd terminal: `ros2 topic echo /robot_1/odom` - Measured position and velocity of the second robot should be continuously printing out in the terminal.

### Tmuxinator example
The same example as above can be easily done using [Tmuxinator](https://github.com/tmuxinator/tmuxinator). (_You must have it installed and also be using custom larics keybindings. If you are using this in Docker, you got everything you need_). By running the command below, multiple terminals will open and run the necessary commands automatically.
```bash
cd <path_to_>/sphero_stage/launch
tmuxinator start -p example.yml
```
You can move between terminals by holding the `Ctrl` key and pressing arrow keys. When done, you can press `Ctrl+b` and then `k` to kill all programs and exit.