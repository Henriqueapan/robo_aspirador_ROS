xhost +local:docker

docker run -it \
    --net=host \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/src:/root/catkin_ws/src:rw" \
    --name ros_gazebo \
    ros-gazebo \
    bash
