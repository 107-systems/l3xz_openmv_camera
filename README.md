<a href="https://107-systems.org/"><img align="right" src="https://raw.githubusercontent.com/107-systems/.github/main/logo/107-systems.png" width="15%"></a>
:floppy_disk: `l3xz_openmv_camera`
==================================
ROS driver for [OpenMV Cam H7 R2](https://openmv.io/collections/cams/products/openmv-cam-h7-r2), including support for thermal vision with [FLIR Lepton adapter module](https://openmv.io/collections/cams/products/flir-lepton-adapter-module).

# Setup

#### Target (Robot)
[Raspberry Pi 4/8 GB](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/), [Buster](https://www.raspberrypi.com/software/operating-systems/#raspberry-pi-os-legacy), [ROS Noetic Ninjemys](https://varhowto.com/install-ros-noetic-raspberry-pi-4/).
#### Host (Devlopment PC)
Ubuntu 20.04 LTS, [ROS Noetic Ninjemys](http://wiki.ros.org/noetic/Installation/Ubuntu).

## Prepare camera
1) Connect camera via USB
2) Copy ```camera_script/main.py``` to the USB drive of the camera
3) Unmount camera
4) Reconnect camera

## Build Node
```bash
source /opt/ros/noetic/setup.bash
# Prepare catkin workspace (only need to be done once)
mkdir -p catkin_ws/src && cd catkin_ws
catkin_make
# Clone this repository
cd src && git clone https://github.com/107-systems/l3xz_openmv_camera && cd ..
# Invoke catkin_make from the catkin workspace root.
catkin_make
```

# Published Topics

| Default name | Type |
| ------------ | ---- |
| /openmv/image_color | sensor_msgs/Image |
| /openmv/image_color_compressed | sensor_msgs/CompressedImage |
| /openmv_camera_info | sensor_msgs/CameraInfo |

# Services

| Default name | Description |
| ------------ | ----------- |
| /openmv/rgb | Set binary values to RGB LED |

# Parameters

| Name | Default | Description |
| ---- | ------- | ----------- |
| image_topic | image_color | RGB image |
| image_queue | 1 | Queue size for RGB image topic |
| info_topic | camera_info | Camera configuration |
| info_queue | 1 | Queue size for camera info topic |
| show_image | false | Show RGB image in OpenCV window |
| port | /dev/ttyACM0 | Serial port of camera |
| rate_hz | 10 | fps |
| frame_id | odom | camera tf frame |
| resolution | QVGA | image resolution |
