# most important library 
import rclpy
from rclpy.node import Node

# message type to send
from sensor_msgs.msg import Image , CompressedImage
from builtin_interfaces.msg import Time

# everything OpenCV related
from cv_bridge import CvBridge
import cv2

from GstOpenCVConverter import GstOpenCVConverter

# The Jetson Nano's GPU has dedicated hardware for video encoding and decoding. 
# GStreamer pipelines allow you to leverage this hardware for efficient video processing.
# Directly accessing the CSI camera with standard OpenCV functions might bypass this acceleration, 
# leading to poor performance (high CPU usage, low frame rates).
# In order to test some pipelines commands you can use the "gst-launch-1.0" command 

# a common pipeline :
# gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1' ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink

# nvarguscamerasrc sensor-id=0:                                                     Captures video from the CSI camera (sensor ID 0).
# 'video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1':   Specifies the video format, resolution, and frame rate.
# nvvidconv:                                                                        Converts the video format using hardware acceleration.
# video/x-raw, format=BGRx:                                                         Changes the video to BGRx format.
# videoconvert:                                                                     Converts the video to BGR.
# appsink:                                                                          Sends the video data to your application.

 
# This node is based upon : 
# https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
# https://github.com/Devanthro/csi_camera/blob/main/csi_camera/ros2_simple_camera.py
# https://automaticaddison.com/how-to-set-up-a-camera-for-nvidia-jetson-nano/

class ImagePublisher(Node):

    def __init__(self, node_name = "jetsonCameraPublisher", *, context = None, cli_args = None, namespace = None, use_global_arguments = True, enable_rosout = True, start_parameter_services = True, parameter_overrides = None, allow_undeclared_parameters = False, automatically_declare_parameters_from_overrides = False):
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        

        pipeline_str = "\
            nvarguscamerasrc ! \
            video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! nvvidconv ! \
            video/x-raw, format=BGRx ! \
            videoconvert ! \
            video/x-raw, format=RGB ! \
            appsink name=sink"
        self.gst_to_frame_converter = GstOpenCVConverter(pipeline_source=pipeline_str)
        self.bridge = CvBridge()

        self.last_image = None
        self.PUBLISH_RATE = 10
        self.IMAGE_READING_RATE = 60

        self.publisher_image_compressed = self.create_publisher(
           msg_type=CompressedImage, 
           topic='camera/image_raw/compressed', 
           qos_profile=10
        )
        
        self.publisher_image = self.create_publisher(
           msg_type=Image, 
           topic='camera/image_raw', 
           qos_profile=10
        )
        
        self.readImage = self.create_timer(
           timer_period_sec=1.0/self.IMAGE_READING_RATE, 
           callback=self.read_image
        )

        self.publishImage = self.create_timer(
           timer_period_sec=1.0/self.PUBLISH_RATE, 
           callback=self.publish_image
        )
            
        
    def read_image(self):
        try:
            self.last_image = self.gst_to_frame_converter.get_frame()
        except Exception as e:
            self.get_logger().info(e)

    def publish_image(self):
        # getting frame from openCV (thus from the camera)
        img_msg_comp = self.bridge.cv2_to_compressed_imgmsg(self.last_image)
        img_msg = self.bridge.cv2_to_imgmsg(self.last_image)
        
        # adding time
        time = Time()
        mgs_time = self.get_clock().now().seconds_nanoseconds()
        time.sec = int(mgs_time[0])
        time.nanosec = int(mgs_time[1])
        img_msg_comp.header.stamp = time
        img_msg.header.stamp = time

        img_msg_comp.header.frame_id = "camera_link_optical"
        img_msg.header.frame_id = "camera_link_optical"

        # publish to the ROS2 topic
        self.publisher_image_compressed.publish(img_msg_comp)
        self.get_logger().info('Publishing video frame compressed')
        self.publisher_image.publish(img_msg_comp)
        self.get_logger().info('Publishing video frame raw')

  
def main(args=None):
  rclpy.init(args=args)
  
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  
  image_publisher.cap.release()
  image_publisher.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()