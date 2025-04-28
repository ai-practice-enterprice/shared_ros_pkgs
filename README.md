# shared_ros_pkgs
all shared pkgs for the robots

This repository contains the following ROS2 (Humble) packages

- camera_image_publisher:
  - GstOpenCVConverter \
    [NOT a ROS2 Node]
  - cam_jetson_publisher \
    [ROS2 Node for the jetson using the GstOpenCVConverter]
  - cam_publisher \
    [ROS2 Node for simulations not requiring special interaction with Gstreamer]
  - test_jetson_camera \
    [some python script that you can run to test out the gi python module]
- midas_node:
  - MidasDetector \
    [NOT a ROS2 Node]
  - detectionV2 \
    [ROS2 Node that subscribes to a CompressedImage topic and publishes the processed result to a Image topic `midas_result`]
- recorder:
  - recorder_node \
    [subscribes to a Image topic and writes all the images as a video using OpenCV's VideoWriter module]
- yolo_ros:
  - yolo_node \
    [ROS2 Node that subscribes to a Image topic and publishes the processed result to a Image topic `yolo_result`]
![global_overview](./assets/ros_shared_pkgs_structure.svg)
