import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge  # Convenient for handling image conversions
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

class CSICameraPublisher(Node):

    def __init__(self):
        super().__init__('csi_camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # Initialize GStreamer
        Gst.init(None)

        # Construct the GStreamer pipeline for CSI camera
        # Adjust the pipeline string based on your camera and desired format
        self.pipeline_str = """
            nvarguscamerasrc sensor-id=0 ! 
            'video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1' ! 
            nvvidconv ! 
            'video/x-raw, format=BGR' ! 
            videoconvert ! 
            appsink name=sink emit-signals=true sync=false
        """
        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')
        self.appsink.connect("new-sample", self.on_new_sample)

        self.bus = self.pipeline.get_bus()
        self.bus.add_signal_watch()
        self.bus.connect("message::error", self.on_error)

        self.pipeline.set_state(Gst.State.PLAYING)
        self.get_logger().info('GStreamer pipeline started.')

    def on_new_sample(self, sink):
        sample = sink.pull_sample()
        if sample:
            buf = sample.get_buffer()
            caps = sample.get_caps()
            structure = caps.get_structure(0)
            width = structure.get_value('width')
            height = structure.get_value('height')
            format_str = structure.get_value('format')

            # Map the buffer to get raw image data
            success, map_info = buf.map(Gst.MapFlags.READ)
            if success:
                ptr = map_info.data
                size = buf.get_size()

                # Create the ROS 2 Image message
                img_msg = Image()
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "camera_frame"  # Adjust as needed
                img_msg.height = height
                img_msg.width = width
                img_msg.encoding = self.bridge.cvtype_to_encoding("bgr8")  # Assuming BGR format
                img_msg.step = width * 3  # Bytes per row (for BGR8)
                img_msg.data = ptr.tobytes()

                self.publisher_.publish(img_msg)
                buf.unmap(map_info)
            return Gst.FlowReturn.OK
        return Gst.FlowReturn.ERROR

    def on_error(self, bus, msg):
        err, debug = msg.parse_error()
        self.get_logger().error(f"GStreamer error: {err} - {debug}")

    def destroy_node(self):
        self.get_logger().info('Stopping GStreamer pipeline.')
        self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = CSICameraPublisher()
    rclpy.spin(camera_publisher)
    camera_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()