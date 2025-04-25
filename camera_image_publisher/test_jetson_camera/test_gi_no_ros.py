import gi
gi.require_version('Gst', '1.0')
from gi.overrides import Gst , GLib

def on_eos(self, bus, msg):
    print("End of stream")
    pipeline.set_state(Gst.State.NULL)

def on_error(self, bus, msg):
    err, debug = msg.parse_error()
    print(f"Error: {err} - {debug}")
    pipeline.set_state(Gst.State.NULL)

Gst.init(None)
pipeline_str = "v4l2src ! videoconvert ! video/x-raw,format=RGB ! appsink name=sink"
# For compressed image:
# pipeline_str = "v4l2src ! videoconvert ! video/x-raw,format=RGB ! jpegenc ! appsink name=sink"
# Other options:
# pipeline_str = "nvarguscamerasrc ! videoconvert ! video/x-raw,format=RGB ! jpegenc ! appsink name=sink"
# pipeline_str = "nvarguscamerasrc ! videoconvert ! video/x-raw,format=(string)BGR ! jpegenc ! appsink name=sink"

pipeline = Gst.parse_launch(pipeline_str)
appsink = pipeline.get_by_name("sink")
bus = pipeline.get_bus()
bus.add_signal_watch()
bus.connect("message::eos", on_eos)
bus.connect("message::error", on_error)

pipeline.set_state(Gst.State.PLAYING)