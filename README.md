&#8598; Click in the upper left corner for Table of Contents

# FER - MRS Course Project
Dear students, welcome to the Multi-Robot Systems course. These instructions will help you set up everything needed for the project you will be working on as a part of the course requirements.

## Intro
In this project you will be developing algorithms for controlling a swarm of Sphero robots, but first, everything will be done in simulation. You are going to use a simple 2D simulator [Stage](https://github.com/rtv/Stage), and specifically, it's bindings to the Robot Operating System (ROS) [stage_ros](http://wiki.ros.org/stage_ros).

> Don't worry if you have never worked with ROS. For this project you will need only the basics, and there is probably someone in your group who already has some experience.

Sphero robots are simulated in Stage as simple omni-directional points (actually squares, but the shape is not important) without mass. This means that they have no inertia, i.e., they can achieve the commanded velocity instantly. Desired velocities are commanded by sending a [geometry_msgs/Twist](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Twist.html) message on `robot_x/cmd_vel` topic where `x` is the number of the robot in simulation (0, 1, 2, ...). At the same time, the simulator is continuously streaming positions and velocities of the robots on topic `robot_x/odom` of type [nav_msgs/Odometry](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/Odometry.html).

## Reporting problems
If you encounter an error that you can't solve yourself or get stuck on some step of the simulation, please open an [Issue](https://github.com/larics/mrs_course/issues) in this repository. Specify your operating system and method of installation, describe your problem, and include the entire output of the command that resulted in error. This will be the quickest way to get feedback and will help other students who may encounter the same error in the future.

## Installation
There are two ways you can set up your computer to run the simulation:

1. **Using Docker** (recommended) - everything you need including ROS, core ROS tools, Stage simulator, and other packages is installed and ready to use.
2. If you **already have ROS** installed, the remaining packages can be installed manually.

### 1) Docker installation (recommended)
Very briefly, Docker is tool that provides simple and efficient way to pack everything needed for a specific application in one container. You can also look at it as a lightweigh virtual machine running on your computer.

Basic information about Docker and its main concepts can be found [here](https://github.com/larics/docker_files/blob/master/Instructions.md), while more detailed instructions and troubleshooting is available [here](https://github.com/larics/docker_files). Of course, you can also take a look at the [official website](https://www.docker.com/). Don't follow any instructions from these links just yet. They are provided as a general overview and reference you can use in the future. Detailed step-by-step instructions are given below.

#### Prerequisites
You must have Ubuntu OS installed on your computer. Ideally, this would be Ubuntu 20.04, but other version should work as well.

#### Step-by-step instructions
Follow these [instructions](https://docs.docker.com/engine/install/ubuntu/) to install Docker engine.

Then follow these [optional steps](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) to manage docker as a non root user. If you skip this, every `docker` command will have to be executed with `sudo`. Skip the _"Note: To run Docker without root privileges, see Run the Docker daemon as a non-root user (Rootless mode)."_ part. This is just a note and we do not need it.

Docker containers are intended to run inside your terminal. In other words, you won't see a desktop like in regular virtual machines. However, graphical applications can still run in the container if you give them permission. To do that, execute
```bash
xhost +local:docker
```
To avoid having to do this every time, we can add that command to our `.profile` file which executes on every login.
```bash
echo "xhost +local:docker > /dev/null" >> ~/.profile
```

Now, let's prepare the docker container that will be used for the project. First, create an empty directory on your computer and position yourself inside of it.
```bash
# Assuming you are in your home folder
mkdir mrs_project
cd mrs_project
```

Clone the repository with Dockerfile in your new directory and position yourself in the correct folder.
```bash
git clone https://github.com/larics/docker_files.git
cd docker_files/ros-noetic/mrs_course_project
```

Build a Docker image. You will see a lot of output. Wait until it's done.
```bash
docker build -t mrs_project_img:latest .
```

Once you have an image, you need a container. The `first_run.sh` script will do everyting for you.
```bash
bash first_run.sh
```

When the script finishes, your terminal prompt should change from
```bash
<your username>@<your hostname>
```
to
```bash
developer@<your hostname>
```
This signals that you are currently "inside" the container.

> Short reminder on commands for working with the container:  
> - Exiting from container - Press `Ctrl+d`
> - Stopping container from the outside (some other terminal) - `docker stop mrs_project`
> - Starting container - `docker start -i mrs_project`
> - Open a new terminal connected to running container - `docker exec -it mrs_project bash`  
> 
> More details are available in the main [docker_files](https://github.com/larics/docker_files) repo and online.

#### Bonus section
The provided Docker image comes with a few preinstalled tools and configs which may simplify your life.

**Tmuxinator** is a tool which allows you to start a tmux session with complex layout and automatically run commands by configuring a simple yaml configuration file. Tmux is a terminal multiplexer - it can run multiple terminal windows inside a single window. This approach is simpler than having to do `docker exec` every time you need a new terminal.

You don't need to write new configuration files for your projects, but some examples will use Tmuxinator. You can move between terminal panes by holding down `Ctrl` key and navigating with arrow keys. Switching between tabs is done with `Shift` and arrow keys. If you have a lot of open panes and tabs in your tmux, you can simply kill everything and exit by pressing `Ctrl+b` and then `k`.

Here are some links: [Tmuxinator](https://github.com/tmuxinator/tmuxinator), [Getting starded with Tmux](https://linuxize.com/post/getting-started-with-tmux/), [Tmux Cheat Sheet](https://tmuxcheatsheet.com/)

**Ranger** is a command-line file browser for Linux. While inside Docker container, you can run the default file browser `nautilus` with a graphical interface, but it is often easier and quicker to view the files directly in the terminal window. You can start ranger with command `ra`. Moving up and down the folders is done with arrow keys and you can exit with a `q`. When you exit, the working directory in your terminal will be set to the last directory you opened while in ranger.

**Htop** is better version of `top` - command line interface task manager. Start it with command `htop` and exit with `q`.

**VS Code** - If you normally use VS Code as your IDE, you can install [Dev Containers](https://code.visualstudio.com/docs/remote/containers#_sharing-git-credentials-with-your-container) extension which will allow you to continue using it inside the container. Simply start the container in your terminal (`docker start -i mrs_project`) and then attach to it from the VS code (open action tray with `Ctrl+Shift+P` and select `Dev Containers: Attach to Running Container`).

### 2) Manual installation (if you already have ROS installed)
> We are assuming that you have ROS Noetic installed.

Even if you have ROS installed, consider using Docker installation as it will simplify debugging and final evaluation, you will learn something new, and the Docker image comes with some extra goodies.

Prerequisites on ROS packages:
```bash
sudo apt install -y \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-map-server \
    ros-${ROS_DISTRO}-stage-ros
```

Prerequisites on custom packages:
```bash
cd <your_workspace>/src
git clone https://github.com/larics/sphero_simulation.git
catkin build
```

## Testing your setup with an example
Now that you have the simulator ready, you can try to move the robots in it. Take a look at the Tmuxinator example in [sphero_stage](https://github.com/larics/sphero_simulation/tree/master/sphero_stage) package. When you start the example, a window with simulation should open. In the middle two terminals you can press keys on your keyboard to send velocity commands to robots. In the bottom two terminals you can see their current position and velocity.

As explained in the Bonus section about Tmuxinator, switching between windows is achieved by holding down `Ctrl` key and pressing arrow keys. If you want to send commands to the robot, one of middle the panels (terminals) must be selected.

Move the robot forwards and backwards with keys `i` and `<`. Rotate left and right with `j` and `l`. For the holonomic mode (strafing), hold down the `Shift` button and press `i`, `<`, `j` or `l`. Try out other keys as well and see what they do.

## Working on your project
The [sphero_stage](https://github.com/larics/sphero_simulation/tree/master/sphero_stage) package is the main package you will be using for setting up and starting your simulation environment. Detailed instructions are available there.

For developing your solution, create a new package. You can write your code in Python or C++.

More packages will be made available as needed, including the instructions on how to use them.
