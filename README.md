# ROS2_Humble_OAKD_LITE

This repository contains all you need to run ROS2 humble with OAKD-LITE and all for the CREATE3 robot on a Raspberry Pi.  I tested it on an Raspberry Pi 4


Example PointCloud ROS2 topic publisher in RVIZ2 :

![image](https://github.com/user-attachments/assets/e57aad29-5ea8-4dd4-baae-fe214805f642)


Example of Depth 2 Scan topic for use as a Lidar data

![image](https://github.com/user-attachments/assets/ad912ac2-f68a-47e4-8736-096946892761)



#On the Raspberry PI

## Installing

git clone https://github.com/mdequanter/ROS2_Humble_OAKD_LITE.git

cd ROS2_Humble_OAKD_LITE

chmod +x docker_build.sh
chmod +x start_docker.sh

./docker_build.sh


The first time building will take some time.

## Use launching

./start_docker.sh

Then in the docker :  python3 depth2PointCloud.py

It will start publishing topics.

# On the Desktop Ubuntu system

git clone https://github.com/mdequanter/ROS2_Humble_OAKD_LITE.git

cd ROS2_Humble_OAKD_LITE

rviz2 -d dept2PointCloud.rviz

