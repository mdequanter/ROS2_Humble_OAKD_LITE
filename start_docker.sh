docker run -it --rm \
  --net=host \
  --privileged \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=1 \
  -v /dev:/dev \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  ros2-custom:create3
