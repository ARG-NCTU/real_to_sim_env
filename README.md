# nctu_simulation_env
This is a ROS package for the some simulation environments (gazebo world).

Then, you can spawn the robot into the world.

![](image/rviz.png)

## Download Mesh
`cd mesh`

`source download_environments.sh`

## Launch Env
### NCTU EE6F
`roslaunch nctu_simulation_env nctu_gazebo.launch world:=EE6F`

![](image/EE6F.png)


### auditorium
`roslaunch nctu_simulation_env nctu_gazebo.launch world:=auditorium`

![](image/auditorium.png)

### urban
`roslaunch nctu_simulation_env nctu_gazebo.launch world:=urban`

![](image/urban.png)

### urban with door

This world also need this ROS package [dynamic_gazebo_models](https://github.com/kuolunwang/dynamic_gazebo_models.git)


`roslaunch nctu_simulation_env nctu_gazebo.launch world:=urban_with_door`

![](image/urban_with_door.png)
