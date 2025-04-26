import gi
gi.require_version('Gst','1.0')
gi.require_version('GstVideo','1.0')
gi.require_version('GstRtspServer','1.0')
from gi.repository import GLib, GObject, Gst, GstVideo, GstRtspServer
import cv2
import numpy as np


# https://idomagor.medium.com/introduction-of-gstreamer-8a513e5478d0

Gst.init(None)
pipeline_str = "nvarguscamerasrc ! video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=30/1 ! nvvidconv ! video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=RGB ! appsink name=sink"

VIDEO_FORMAT = "RGB"
GST_VIDEO_FORMAT = GstVideo.VideoFormat.from_string(VIDEO_FORMAT)

try:
    # callback function to handle new samples.
    def on_new_sample(appsink):
        sample = appsink.emit("pull-sample")
        if sample:
            caps = sample.get_caps()
            # sample.get_buffer()
            # Extract the width and height info from the sample's caps
            height = caps.get_structure(0).get_value("height")
            width = caps.get_structure(0).get_value("width")
            # Get the actual data
            buffer = sample.get_buffer()

            print(f"width and height: {width} , {height}")
            print(f"buffer size {buffer.get_size()}")

            # Get read access to the buffer data
            success, map_info = buffer.map(Gst.MapFlags.READ)
            if not success:
                raise RuntimeError("Could not map buffer data!")
            
            frame_data = np.frombuffer(map_info.data,dtype=np.uint8)
            frame = frame_data.reshape((height, width, 3))
            print(frame)
            

    # callback to handle stream end or stream error
    def bus_call(bus, message, loop):
        if message.type == Gst.MessageType.EOS:
            print("End of stream")
            loop.quit()
        elif message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}, {debug}")
            loop.quit()
        return True
    
    pipeline = Gst.parse_launch(pipeline_str)
    bus = pipeline.get_bus()
    pipeline.set_state(Gst.State.PLAYING)
    print("Streaming video from /dev/video0 to your screen. Press Ctrl+C to stop.")
    
    # add a "sink" (output) to the pipeline.
    # video_sink = Gst.ElementFactory.make("appsink", "sink")
    video_sink = pipeline.get_by_name("sink")
    # Set properties to enable signal emission and asynchronous processing.
    video_sink.set_property("emit-signals", True)
    video_sink.set_property("sync", False)
    video_sink.connect("new-sample", on_new_sample)
    # pipeline.add(video_sink)

    mainloop = GLib.MainLoop()
    bus.add_signal_watch()
    bus.connect("message::eos", bus_call, mainloop)
    bus.connect("message::error", bus_call, mainloop)

    try:
        print("Running stream.")
        mainloop.run()
    except KeyboardInterrupt:
        print("Stopping stream.")
    finally:
        # Clean up
        pipeline.set_state(Gst.State.NULL)

except Exception as e:
    print(f"An error occurred: {e}")

