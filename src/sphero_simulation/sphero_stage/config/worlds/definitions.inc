define robot position
(
#	block
#	(
#			points 3
#			point[0] [1 0.5]
#			point[1] [0 1]
#			point[2] [0 0]
#			z [0 1]
#	)

	size [0.075 0.075 0.075]
	origin [0 0 0 0]

	gui_nose 1
	drive "omni"
	localization "gps"
	localization_origin [0 0 0 0]
	update_interval  10 # update timestep in milliseconds

	# [ xmin xmax ymin ymax zmin zmax amin amax ]
	velocity_bounds [-1 1 -1 1 -1 1 -90 90 ]
	acceleration_bounds [-0.01 0.01 -0.01 0.01 -1 1 -90 90]
	# Acceleration bounds are only applied when using acceleration commands.
	# stage_ros2 does not currently expose this functionality. To add it, one
	# should add a function like
	#
	# ```void StageNode::Vehicle::callback_cmd(const geometry_msgs::msg::Twist::SharedPtr msg)```
	#
	# at the end of this file https://github.com/tuw-robotics/stage_ros2/blob/jazzy/src/vehicle.cpp
	# and then use `SetAcceleration` instead of `SetVelocity` in the body of
	# that function, following https://github.com/rtv/Stage/blob/281ed731ed56b23d0879a2c765d4a20824a6d58a/libstage/model_position.cc#L652
)

define floorplan model
(
	# sombre, sensible, artistic
	color "gray30"

	# most maps will need a bounding box
	boundary 1

	gui_nose 1
	gui_grid 0
	gui_outline 0

	update_interval  10 # update timestep in milliseconds
)

# set the resolution of the underlying raytrace model in meters
resolution 0.01

interval_sim 10  # simulation timestep in milliseconds

window
(
	size [ 430.000 480.000 ]
	center [ 0 -0.3 ]
	rotate [ 0.000 0 ]
	scale 42
)