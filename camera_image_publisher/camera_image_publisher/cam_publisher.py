# most important library 
import rclpy
from rclpy.node import Node

# message type to send
from sensor_msgs.msg import Image

# everything OpenCV related
from cv_bridge import CvBridge
import cv2
 
# This node is based upon : https://automaticaddison.com/getting-started-with-opencv-in-ros-2-foxy-fitzroy-python/
class ImagePublisher(Node):

    def __init__(self, node_name, *, context = None, cli_args = None, namespace = None, use_global_arguments = True, enable_rosout = True, start_parameter_services = True, parameter_overrides = None, allow_undeclared_parameters = False, automatically_declare_parameters_from_overrides = False):
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        
        self.publisher_ = self.create_publisher(
           msg_type=Image, 
           topic='camera/image_raw', 
           qos_profile=10
        )
        
        timer_period = 0.1 
        self.timer = self.create_timer(
           timer_period_sec=timer_period, 
           callback=self.timer_callback
        )
            
        # Create a VideoCapture object
        # The argument '0' gets the default webcam.
        self.cap = cv2.VideoCapture(0)
            
        self.br = CvBridge()
   
    def timer_callback(self):
        ret, frame = self.cap.read()

        if ret == True:
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
            self.get_logger().info('Publishing video frame')
    
  
def main(args=None):
  rclpy.init(args=args)
  
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  
  image_publisher.destroy_node()
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()