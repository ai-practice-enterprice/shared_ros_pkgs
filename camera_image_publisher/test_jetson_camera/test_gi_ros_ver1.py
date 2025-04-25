import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import numpy as np
import threading

class CSICameraPublisher(Node):
    def __init__(self):
        super().__init__('csi_camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        Gst.init(None)
        self.pipeline = Gst.parse_launch(
            'nvarguscamerasrc ! '
            'video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! '
            'nvvidconv ! video/x-raw, format=BGRx ! '
            'videoconvert ! video/x-raw, format=BGR ! '
            'appsink name=appsink emit-signals=true sync=false max-buffers=1 drop=true'
        )

        self.appsink = self.pipeline.get_by_name('appsink')
        self.appsink.connect('new-sample', self.on_new_sample)

        self.pipeline.set_state(Gst.State.PLAYING)

        # Needed for GStreamer callbacks
        self.mainloop = GLib.MainLoop()
        self.glib_thread = threading.Thread(target=self.mainloop.run)
        self.glib_thread.start()

    def on_new_sample(self, sink):
        sample = sink.emit('pull-sample')
        buf = sample.get_buffer()
        caps = sample.get_caps()

        # Get image size
        width = caps.get_structure(0).get_value('width')
        height = caps.get_structure(0).get_value('height')

        # Extract frame data
        success, map_info = buf.map(Gst.MapFlags.READ)
        if success:
            frame = np.frombuffer(map_info.data, np.uint8)
            frame = frame.reshape((height, width, 3))
            buf.unmap(map_info)

            # Convert to ROS2 Image and publish
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            self.publisher_.publish(msg)
        return Gst.FlowReturn.OK

    def destroy(self):
        self.pipeline.set_state(Gst.State.NULL)
        self.mainloop.quit()
        self.glib_thread.join()

def main(args=None):
    rclpy.init(args=args)
    node = CSICameraPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()