import cv2
import numpy as np
import gi
gi.require_version('Gst','1.0')
gi.require_version('GstVideo','1.0')
from gi.repository import Gst , GLib


# reference:
# https://gstreamer.freedesktop.org/documentation/index.html?gi-language=python
# https://stackoverflow.com/questions/69441058/setting-up-gstreamer-pipeline-in-python
# https://www.youtube.com/watch?app=desktop&v=HDY8pf-b1nA
# https://gstreamer.freedesktop.org/documentation/tools/gst-launch.html?gi-language=c

class GstOpenCVConverter():
    def __init__(self,pipeline_source: str,show_opencv_window: bool = False):
        Gst.init()
        self.mainloop = GLib.MainLoop()

        self.setup_source_pipeline(pipeline_source)
        
        # init 1st frame
        self.frame = None
        self.show_opencv_window = show_opencv_window
 
        try:
            print("Running stream.")
            self.mainloop.run()
        except KeyboardInterrupt:
            print("Stopping stream.")
        finally:
            # Clean up
            self.pipeline.set_state(Gst.State.NULL)
            cv2.destroyAllWindows()

    def show_frame(self):
        if self.frame is not None:
            cv2.imshow("Camera", self.frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.mainloop.quit()
            # stop calling this function
            return False
        # continue calling this function 
        return True 

    def get_frame(self,frame):
        return self.frame

    def setup_source_pipeline(self,pipeline_str):
        self.pipeline = Gst.parse_launch(pipeline_str)
        self.pipeline.set_state(Gst.State.PLAYING)
        print("# ----------- Streaming video. Press Ctrl+C to stop ----------- #")
        
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message::eos", self.bus_call, self.mainloop)
        bus.connect("message::error", self.bus_call, self.mainloop)
        
        video_sink = self.pipeline.get_by_name("sink")
        # Set properties to enable signal emission and asynchronous processing.
        video_sink.set_property("emit-signals", True)
        video_sink.set_property("sync", False)
        video_sink.connect("new-sample", self.on_new_sample)

    # ---------------- callback  functions for Gstreamer (events) ---------------- # 
    # callback function to handle new samples when send to a "appsink" (so when the output is this application)
    def on_new_sample(self,appsink):
        # a sample is like a frame in OpenCV except that you can use 
        sample = appsink.emit("pull-sample")
        if sample is None:
            return Gst.FlowReturn.ERROR
        # the "caps" (capture) contains all the info about your video frame
        # it does NOT contain the frame's data but does contain the width , height , framerate , nbr of channels, etc...
        caps = sample.get_caps()
        # Extract the width and height info from the sample's caps
        height = caps.get_structure(0).get_value("height")
        width = caps.get_structure(0).get_value("width")
        print(f"width and height: {width} , {height}")

        # so the actual frame's data is passed as a buffer (a 1D array)
        # depending on your width , height , channels (RGB , RGBA , Greyscale, etc...)
        # the buffer will be : width*height*len(channels)   long so that means we can reorder
        # it into a multidimensional array with numpy as long as we know these 3 variables
        buffer = sample.get_buffer()
        print(f"buffer size {buffer.get_size()}")

        # Get read access to the buffer data
        success, map_info = buffer.map(Gst.MapFlags.READ)
        if not success:
            raise RuntimeError("Could not map buffer data!")
        # using numpy's frombuffer function we can read in a 1D array , and convert it into a numpy array  
        # however the data we receive needs to be assigned a data type => since a OpenCV frame in RGB requires int values ranging from [0 ; 255]
        # then we convert into a Uint8 (unsigned integer 8 bits)   
        frame_data = np.frombuffer(map_info.data,dtype=np.uint8)
        # then we transform the 1D array into a multidimensional array (height*width*len(nbr of channels) => RGB = 3 channels red , green , blue)
        self.frame = frame_data.reshape((height, width, 3))
        
        if self.frame is not None:
            self.frame = cv2.cvtColor(self.frame, cv2.COLOR_RGB2BGR)
            if self.frame is not None and self.recording:

                frame_bytes = self.frame.tobytes()
                buf = Gst.Buffer.new_allocate(None, len(frame_bytes), None)
                buf.fill(0, frame_bytes)
                duration = Gst.util_uint64_scale_int(1, Gst.SECOND, int(self.record_fps))  # frame duration
                buf.duration = duration
                buf.pts = self.timestamp
                buf.dts = self.timestamp
                self.timestamp += duration
                if self.recording:
                    self.push_frame_async(buf)
            
            return Gst.FlowReturn.OK
        else:
            return Gst.FlowReturn.ERROR

    # callback to handle stream end or stream error
    def bus_call(self,bus, message, loop):
        if message.type == Gst.MessageType.EOS:
            print("End of stream")
            loop.quit()
        elif message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}, {debug}")
            loop.quit()
        return True