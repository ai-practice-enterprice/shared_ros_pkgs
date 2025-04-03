import cv2

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

cap = cv2.VideoCapture(gstreamer_pipeline(), cv2.CAP_GSTREAMER)

# MAIN LOOP ============================================================================
if cap.isOpened():
    while True:
        ret, frame = cap.read()
        if ret:
            cv2.imshow("Camera", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            print("Error: Could not read frame.")
            break
    cap.release()
    cv2.destroyAllWindows()
else:
    print("Error: Could not open camera.")