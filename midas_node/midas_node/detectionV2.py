import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16
from sensor_msgs.msg import Image , CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from MidasDetector import MidasDetector , ModelType


class MidasNode(Node):
    def __init__(self, node_name = "MidasNode", *, context = None, cli_args = None, namespace = None, use_global_arguments = True, enable_rosout = True, start_parameter_services = True, parameter_overrides = None, allow_undeclared_parameters = False, automatically_declare_parameters_from_overrides = False):
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        self.get_logger().info(f"--- booting up {self.get_name()} ---")
        
        self.SIMULATION = True
        self.DEBUG = True
        
        # we init the Midas detector
        self.midas = MidasDetector(ModelType.MIDAS_SMALL)
        # and the famous CV_Bridge
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            msg_type=Image,
            topic='camera/image_raw',
            callback=self.camera_callback,
            qos_profile=10
        )

        if self.DEBUG:

            self.img_pub_roi = self.create_publisher(
                msg_type=Image, 
                topic="midas_result/roi", 
                qos_profile=10
            )

        self.img_pub = self.create_publisher(
            msg_type=Image, 
            topic="midas_result", 
            qos_profile=10
        )

        self.detection_result_pub = self.create_publisher(Int16, 'midas_detection', 10)
        self.previous_status = None
        self.get_logger().info(f"--- booting up complete ---")


    def camera_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data)

        # the threshold is hardcoded inside the node but maybe this can be 
        # implemented as a service accessible for other nodes such that the
        # FSMNaviator can influence it's search range 
        depth_colored, depth_raw , danger , roi_dim = self.midas.predict_depth(img)
        
        if self.SIMULATION:
            img_msg = self.bridge.cv2_to_imgmsg(
                depth_colored,
                encoding="rgb8"
            )  
            if self.DEBUG:
                img_msg_roi = self.bridge.cv2_to_imgmsg(
                    depth_colored[roi_dim[1]:roi_dim[1]+roi_dim[3], roi_dim[0]:roi_dim[0]+roi_dim[2]],
                    encoding="rgb8"
                )  
                self.img_pub_roi.publish(img_msg_roi)
        else:
            img_msg = self.bridge.cv2_to_imgmsg(
                depth_colored,
            )  
            if self.DEBUG:
                img_msg_roi = self.bridge.cv2_to_imgmsg(
                    depth_colored[roi_dim[1]:roi_dim[1]+roi_dim[3], roi_dim[0]:roi_dim[0]+roi_dim[2]],
                )  
                self.img_pub_roi.publish(img_msg_roi)
        self.img_pub.publish(img_msg)



        # Make New Message To send feedback to the FSMNavigator node of the Robot
        result = Int16()
        if danger:
            result.data = 0
            if self.previous_status != "STOP":
                # self.get_logger().info("STOP")
                self.previous_status = "STOP"
        else:
            result.data = 1
            if self.previous_status != "GO":
                # self.get_logger().info("GO")
                self.previous_status = "GO"

        self.detection_result_pub.publish(result)


def main(args=None):
    rclpy.init(args=args)
    
    midas_node = MidasNode()
    rclpy.spin(midas_node)

    midas_node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()