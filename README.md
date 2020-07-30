# docker_detectron2

A docker image with the detectron2 code.<br/>
A ROS wrapper is included. <br/>
Tested under Ubuntu 18.04 with Docker, Nvidia-Docker, ROS installed.

## camera
Start your camera driver.
Change the image topic in `detectron2_ros.launch` file to your topic.


## detectron2
In this folder:
`docker run --gpus all -it --rm --net host --env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix:rw --mount type=bind,source=$PWD,target=/detectron2 -w /detectron2 registry.gitlab.com/haiandaidi/docker_detectron2:2020_05_16`

`cd catkin_ws`

`source /opt/ros/melodic/setup.bash`

`catkin_make`

`source devel/setup.bash`

`roslaunch detectron2_ros detectron2_ros.launch`

The result can be visualzied from the topic of `/detectron2_ros/visualization` using Rviz.

## synchronizer
`roslaunch detectron2_sync detectron2_sync.launch`