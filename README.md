# Como rodar

## No host

`xhost +local:docker`

## No container

1. `roscore`
2. `roslaunch gazebo_ros empty_world.launch`
3. `roslaunch p3dx_gazebo p3dx.launch`
4. `rosrun begginner_tutorials teste.py`