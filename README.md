# ROS2_Humble_OAKD_LITE

This repository provides a complete setup to run ROS 2 Humble with the OAK-D Lite depth camera and the iRobot Create® 3 robot on a Raspberry Pi 4. It includes Docker-based deployment, sample scripts for publishing depth and point cloud data, and RViz2 configurations for easy visualization. The project is designed to facilitate integration of real-time vision and navigation capabilities on low-power embedded platforms. Tested and verified on a Raspberry Pi 4, this setup enables rapid prototyping and exploration with the Create3 robot using advanced depth perception tools.

The advantage of this repository is, that you can install this docker on a standard Raspberry Pi 4 64 bit OS.  Without having to install ROS native.  The performance is verry good!

![image](https://github.com/user-attachments/assets/75e48173-512b-4595-bafd-ac41725c7f65)



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


# 🐢 Getting Started with iRobot Create® 3 (TurtleBot4 Base)

This guide explains how to get started with the **iRobot Create® 3** robot, which serves as the base for the **TurtleBot4**. It focuses on using the Create3 independently, without a Raspberry Pi connected.

---

## ✅ Prerequisites

1. Connect your **Ubuntu system** to the **same network** as the Create3 robot.
2. Open the web interface of your Create3 at:  
   ```
   http://<IP_OF_CREATE3>
   ```
3. Ensure your Create3 has the correct configuration.

---

## 🔍 Test the Connection

Check if your system can reach the robot:

```bash
ping <IP_OF_CREATE3>
```

Check available topics:

```bash
ros2 topic list
```

You should see topics like:

```
/battery_state
/cliff_intensity
/cmd_audio
/cmd_lightring
/cmd_vel
/dock_status
/hazard_detection
/imu
/interface_buttons
/ir_intensity
/ir_opcode
/kidnap_status
/mobility_monitor/transition_event
/mouse
/odom
/parameter_events
/robot_state/transition_event
/rosout
/slip_status
/static_transform/transition_event
/stop_status
/tf
/wheel_status
/wheel_ticks
/wheel_vels
```

> ⚠️ If you don't see these topics, start debugging your system setup before continuing.

---

## 🦾 Control the Robot via ROS 2

### Drive with Velocity

Publish a `Twist` message to drive the robot:

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

### Drive a Specific Distance

```bash
ros2 action send_goal /drive_distance irobot_create_msgs/action/DriveDistance "{distance: 0.5, max_translation_speed: 0.15}"
```

---

### Rotate an Angle

```bash
ros2 action send_goal /rotate_angle irobot_create_msgs/action/RotateAngle "{angle: 1.57, max_rotation_speed: 0.5}"
```

---

### Drive an Arc

```bash
ros2 action send_goal /drive_arc irobot_create_msgs/action/DriveArc "{angle: 1.57, radius: 0.3, translate_direction: 1, max_translation_speed: 0.3}"
```

---

### Wall Follow

```bash
ros2 action send_goal /wall_follow irobot_create_msgs/action/WallFollow "{follow_side: 1, max_runtime: {sec: 1, nanosec: 0}}"
```

---

### Navigate to a Position

```bash
ros2 action send_goal /navigate_to_position irobot_create_msgs/action/NavigateToPosition "{
  achieve_goal_heading: true,
  goal_pose: {
    pose: {
      position: {x: 1.0, y: 0.2, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```

---

## 🏠 Docking and Undocking

### Undock

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock "{}"
```

### Dock

```bash
ros2 action send_goal /dock irobot_create_msgs/action/Dock "{}"
```

---

## 🎮 Teleoperation

You can control the robot with your keyboard using:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 📎 Helpful Tips

- Use `source /opt/ros/humble/setup.bash` to set up your ROS 2 environment before running commands.
- This guide assumes you're using **ROS 2 Humble**.
- Replace `<IP_OF_CREATE3>` with your robot’s actual IP address.

---

## 📚 Resources

- [iRobot Create® 3 Documentation](https://iroboteducation.github.io/create3_docs/)
- [ROS 2 Humble Docs](https://docs.ros.org/en/humble/index.html)

---

Happy coding! 🤖✨

