import cv2
from cv2 import VideoWriter

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=3280,
    capture_height=2160,
    display_width=3840,
    display_height=2160,
    framerate=20,
    flip_method=1,
):
    return (
        "nvarguscamerasrc sensor_mode=1 sensor-id=%d ! "
        "video/x-raw(memory:NVMM), width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "nvvidconv flip-method=%d ! "
        "video/x-raw, width=(int)%d, height=(int)%d, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! appsink"
        % (
            sensor_id,
            capture_width,
            capture_height,
            framerate,
            flip_method,
            display_width,
            display_height,
        )
    )

def init_VideoWriter(cap: cv2.VideoCapture):
    try:

        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        FPS = cap.get(cv2.CAP_PROP_FPS)

        video_format = 'mp4'
        video_filename = 'recorded_footage.' + video_format

        video_writer = cv2.VideoWriter(
            filename=video_filename, 
            fourcc=VideoWriter.fourcc(*'mkv'), 
            fps=FPS, 
            frameSize=(width, height)
        )

        # might be required instead
        # https://forums.developer.nvidia.com/t/displaying-to-the-screen-with-opencv-and-gstreamer/140648/9
        gst_out = "appsrc ! videoconvert ! video/x-raw,format=BGR ! nvvidconv ! nvv4l2h264enc insert-sps-pps=1 ! h264parse ! matroskamux ! filesink location=output.mkv"
        
        video_writer = cv2.VideoWriter(
            gst_out, 
            cv2.CAP_GSTREAMER,
            fps=FPS, 
            frameSize=(width, height)
        )
        cv2.CAP_GSTREAMER
    
        print('initialized video writer')
        return video_writer
    
    except Exception as e:
        print('Error initializing video writer: %s' % str(e))


cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

# MAIN LOOP ============================================================================
if cap.isOpened():
    video_writer = None

    while True:
        ret, frame = cap.read()

        if video_writer is None:
            video_writer: cv2.VideoWriter = init_VideoWriter()

        if ret:
            
            cv2.imshow("Camera", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            try:
                video_writer.write(frame)
            except Exception as e:
                print(('Error processing image: %s' % str(e)))

        else:
            print("Error: Could not read frame.")
            break
        
    video_writer.release()
    cap.release()
    cv2.destroyAllWindows()
else:
    print("Error: Could not open camera.")