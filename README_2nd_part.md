# FER - MRS Course Project - 2nd part
In the second part of the project, you will apply already implemented algorithms based on Reynolds rules to control Sphero robots in the real world. Just like in the simulation part, we have provided all the required packages for running the robots in [sphero_robot](https://github.com/larics/sphero_robot).

## Installation
Again, there are two ways you can set up your computer to run the simulation:

1. **Using Docker** (recommended)
2. If you **already have ROS** installed, the remaining packages can be installed manually.

### 1) Docker installation (recommended)
Docker images for both parts are very similar so we are simply using the updated image from the first part of the project. This means you will have access to the simulator If you followed the last instructions exactly, you should be able to set everything up as follows.

Navigate to the folder where you originally cloned the `docker_files` repository.
```bash
# Assuming you are in your home folder
cd mrs_project
```

Update the repository with Dockerfile by pulling the new version.
```bash
cd docker_files/ros-noetic/mrs_course_project
git pull
```

Build a Docker image. We have intentionally renamed the image to be different from the first part, but you can use the same name if you'd like (you will also need to modify the first_run script in that case). **Be careful**, by doing so, you might lose the original image.
```bash
docker build -t mrs_project_img_2:latest .
```

Once you have the image, you need a container. The `first_run.sh` script will do everything for you. Again, everything inside is renamed to keep your work safe. The new script does exactly the same as the old one but uses different image and container names and mounts the required volume for accessing the Bluetooth.
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

#### Cleaning out the old image and container.
If you lack disk space on your laptop, you can remove the old image and container. **But first, make sure all your work is saved on some remote repository or local (non-docker) file system.**

To remove the container: `docker rm <container name>`

To remove the image: `docker rmi <image name>`

### 2) Manual installation (if you already have ROS installed)
> We are assuming that you have ROS Noetic installed.

> Also note: manually installing packages in the existing docker container will **not** work - there are additional steps included while building the new image.

Prerequisites on Ubuntu packages:
```bash
sudo apt install -y
    dbus \
    libbluetooth-dev \
    bluetooth \
    blueman \
    bluez \
    libusb-dev \
    bluez-hcidump \
    bluez-tools \
    libglib2.0-dev
```

Prerequisites on ROS packages:
```bash
sudo apt install -y \
    ros-${ROS_DISTRO}-joy \
    ros-${ROS_DISTRO}-rviz
```

Prerequisites on Python packages:
```bash
pip3 install bluepy bleak numpy imutils opencv-python ruamel.yaml
```

Prerequisites on custom packages:
```bash
cd <your_workspace>/src
git clone --recursive https://github.com/larics/sphero_robot.git
git clone https://github.com/mkrizmancic/spherov2.py.git
cd spherov2.py && git checkout devel
pip3 install .
catkin build
```

## Testing your setup with an example
All the functionalities of the `sphero_robot` repository are well documented in their respective packages. However, for your convenience, here is a quick start guide.

### 1) Start the Sphero driver
#### Option a): driver_v2 (the new one)
Make sure to start `roscore`.  
Modify the list of Sphero robots in `cfg/sphero_addresses.txt`  
Run the driver: `rosrun sphero_driver_v2 drivers.launch.py n` where n is the number of robots.  
You should see the available topics when you `rostopic list`.

#### Option b): sprk_ros (the old one)
Modify the list of Sphero robots in `cfg/sphero_addresses.txt`  
Run the driver: `roslaunch sphero_sprk_ros drivers.launch num_of_robots:=n` where n is the number of robots.  
You should see the available topics when you `rostopic list`.

### 2) Control the Spheros with a joystick.
Plug the joystick USB stick into your laptop.  
In a new terminal run `roslaunch sphero_bringup only_joystick.launch`  
Have fun.

### 3) Try out localization.
Plug in webcam USBs to your laptop. Watch out for their order. The camera which is connected first will get a lower index.  
Note the camera access paths by looking at `ls /dev/video*`.  
If needed, modify the config file in `sphero_localization` (the north camera must be first in the list).  
Line up the Sphero(s) with the fixed coordinate frame - by default, y is pointing towards north and x towards east, with origin in the lower left corner (south-west).  
Instead of the joystick launch file, run `roslaunch sphero_bringup webcam_joystick.launch`

### 4) Your solution.
Instead of the webcam+joystick launch file, make your own launch file. Use `webcam_ext.launch` as a template.  
This should start your joystick as well as the custom controller. The joystick is disabled by default. While it's disabled, it's not sending any data and the external controller is active. If the Spheros start going crazy, just re-enable the joystick and they will stop. To make such action possible, you need to send the commanded velocity to `ext_vel` instead of `cmd_vel`.
