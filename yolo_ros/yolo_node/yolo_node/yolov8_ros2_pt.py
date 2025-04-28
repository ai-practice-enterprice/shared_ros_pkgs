#!/usr/bin/env python3
# for ultralytics => because we're running on Ubuntu 22.04  
# we have to carefully select a numpy version that can be used 
# by ultralytics and other libs that ultralytics depends on. Depending on how you installed ultralytics:
# => run => sudo pip install -U numpy==1.26.4
# => or run => python3 -m pip install numpy==1.26.4
from ultralytics import YOLO

import rclpy
import os
import cv2

from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

from ament_index_python.packages import get_package_share_directory

class Camera_subscriber(Node):

    def __init__(self,node_name='camera_subscriber',namespace="",cli_args=""):
        super().__init__(node_name=node_name,namespace=namespace,cli_args=cli_args)

        # retrieve the models weights which is located in the share directory when we build this package with colcon
        package_share_dir = get_package_share_directory('yolo_node')
        best_pt_path = os.path.join(package_share_dir, 'weights', 'best.pt')
        # same for the classes
        classes_path = os.path.join(package_share_dir, 'classes', 'classes.txt')
        
        # we init the YOLO model
        self.model = YOLO(best_pt_path)
        # and the famous CV_Bridge to convert the images to a image format for OpenCV (see below) 
        self.bridge = CvBridge()

        #Load classes
        with open(classes_path, "r") as f:
            self.classes = [line.strip() for line in f.readlines()]

        self.yolov8_inference = Yolov8Inference()

        self.img_pub = self.create_publisher(Image, "yolo_result", 1)

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.camera_callback,
            10)


    def camera_callback(self, data):

        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)
        #Draw bounding boxes and labels
        for result in results:
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                conf = float(box.conf[0])
                class_id = int(box.cls[0])
                label = self.classes[class_id]

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, f"{label} {conf:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        annotated_frame = results[0].plot()
        img_msg = self.bridge.cv2_to_imgmsg(annotated_frame)  

        self.img_pub.publish(img_msg)

        self.yolov8_inference.yolov8_inference.clear()


def main():
    rclpy.init(args=None)
    
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)

    rclpy.shutdown()


if __name__ == '__main__':
    main()