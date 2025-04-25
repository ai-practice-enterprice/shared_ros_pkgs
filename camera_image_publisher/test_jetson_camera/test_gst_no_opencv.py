import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage

# if not installed:
# run => sudo apt install python3-gst-1.0
# or pip install PyGObject
import gi
gi.require_version('Gst', '1.0')
from gi.overrides import Gst

class GStreamerPublisher(Node):
    def __init__(self, node_name = "gstreamer_publisher", *, context = None, cli_args = None, namespace = None, use_global_arguments = True, enable_rosout = True, start_parameter_services = True, parameter_overrides = None, allow_undeclared_parameters = False, automatically_declare_parameters_from_overrides = False):
        super().__init__(node_name, context=context, cli_args=cli_args, namespace=namespace, use_global_arguments=use_global_arguments, enable_rosout=enable_rosout, start_parameter_services=start_parameter_services, parameter_overrides=parameter_overrides, allow_undeclared_parameters=allow_undeclared_parameters, automatically_declare_parameters_from_overrides=automatically_declare_parameters_from_overrides)
        
        self.FPS = 20

        self.publisher_image = self.create_publisher(Image, 'camera/image_raw', 10)
        self.timer = self.create_timer(
            timer_period_sec=1/self.FPS,
            callback=self.publish_frame,
        )
        
        Gst.init(None)
        self.pipeline_str = "v4l2src ! videoconvert ! video/x-raw,format=RGB ! appsink name=sink"
        # For compressed image:
        # self.pipeline_str = "v4l2src ! videoconvert ! video/x-raw,format=RGB ! jpegenc ! appsink name=sink"
        # Other options:
        # self.pipeline_str = "nvarguscamerasrc ! videoconvert ! video/x-raw,format=RGB ! jpegenc ! appsink name=sink"
        # self.pipeline_str = "nvarguscamerasrc ! videoconvert ! video/x-raw,format=(string)BGR ! jpegenc ! appsink name=sink"
        
        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.appsink = self.pipeline.get_by_name("sink")
        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message::eos", self.on_eos)
        self.bus.connect("message::error", self.on_error)

        self.pipeline.set_state(Gst.State.PLAYING)

    def publish_frame(self):
        sample = self.appsink.emit('pull-sample')
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width = structure.get_value("width")
            height = structure.get_value("height")
            format_str = structure.get_value("format")

            # Publish as sensor_msgs/msg/Image
            msg_image = Image()
            msg_image.header.stamp = self.get_clock().now().to_msg()
            msg_image.header.frame_id = "camera_frame"  # Adjust as needed
            msg_image.height = height
            msg_image.width = width
            msg_image.encoding = self.gst_format_to_ros_encoding(format_str)
            msg_image.step = width * self.get_bytes_per_pixel(format_str)
            mapped_buffer = buf.map_readable()
            msg_image.data = mapped_buffer.data.tobytes()
            buf.unmap(mapped_buffer)
            self.publisher_image.publish(msg_image)

            # If using compressed pipeline, publish as sensor_msgs/msg/CompressedImage
            # if "jpegenc" in self.pipeline_str:
            #     msg_compressed = CompressedImage()
            #     msg_compressed.header.stamp = self.get_clock().now().to_msg()
            #     msg_compressed.header.frame_id = "camera_frame"
            #     msg_compressed.format = "jpeg"
            #     mapped_buffer = buf.map_readable()
            #     msg_compressed.data = mapped_buffer.data.tobytes()
            #     buf.unmap(mapped_buffer)
            #     self.publisher_compressed.publish(msg_compressed)

    def gst_format_to_ros_encoding(self, gst_format):
        if gst_format == "RGB":
            return "rgb8"
        elif gst_format == "BGR":
            return "bgr8"
        elif gst_format == "GRAY8":
            return "mono8"
        # Add more mappings as needed
        return ""

    def get_bytes_per_pixel(self, gst_format):
        if gst_format in ["RGB", "BGR"]:
            return 3
        elif gst_format == "GRAY8":
            return 1
        return 0

    def on_eos(self, bus, msg):
        print("End of stream")
        self.pipeline.set_state(Gst.State.NULL)

    def on_error(self, bus, msg):
        err, debug = msg.parse_error()
        print(f"Error: {err} - {debug}")
        self.pipeline.set_state(Gst.State.NULL)

def main(args=None):
    rclpy.init(args=args)
    gstreamer_publisher = GStreamerPublisher()
    rclpy.spin(gstreamer_publisher)
    gstreamer_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()