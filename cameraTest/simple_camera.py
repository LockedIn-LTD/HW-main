import cv2
# Import the numpy library for array manipulation
import numpy as np

# --- GStreamer Pipeline Function (Unchanged - Already Optimal) ---

def gstreamer_pipeline(
    sensor_id=0,
    capture_width=1920,
    capture_height=1080,
    display_width=640, # Adjusted for combined view
    display_height=360, # Adjusted for combined view
    framerate=30,
    flip_method=0,
):
    # This pipeline is already optimized:
    # nvarguscamerasrc -> ISP
    # nvvidconv       -> VIC/JPEG engine for scaling/color conversion
    return (
        "nvarguscamerasrc sensor-id=%d ! "
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

# --- Optimized show_two_cameras Function ---

def show_two_cameras_optimized():
    window_title = "Dual CSI Cameras (Optimized)"
    
    # Define camera display dimensions
    display_w = 640  
    display_h = 360

    # 1. Open both video capture objects
    # GStreamer pipelines leverage ISP/VIC for accelerated capture and scaling
    video_capture_0 = cv2.VideoCapture(
        gstreamer_pipeline(sensor_id=0, display_width=display_w, display_height=display_h), 
        cv2.CAP_GSTREAMER
    )
    video_capture_1 = cv2.VideoCapture(
        gstreamer_pipeline(sensor_id=1, display_width=display_w, display_height=display_h), 
        cv2.CAP_GSTREAMER
    )

    if video_capture_0.isOpened() and video_capture_1.isOpened():
        try:
            cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
            
            # --- Main Loop ---
            while True:
                ret_val_0, frame_0 = video_capture_0.read()
                ret_val_1, frame_1 = video_capture_1.read()
                
                if not (ret_val_0 and ret_val_1):
                    print("Error: Could not read frames from both cameras.")
                    break

                
                combined_frame = np.concatenate((frame_0, frame_1), axis=1)

                
                if cv2.getWindowProperty(window_title, cv2.WND_PROP_AUTOSIZE) >= 0:
                    cv2.imshow(window_title, combined_frame)
                else:
                    break 
                
                keyCode = cv2.waitKey(10) & 0xFF
                if keyCode == 27 or keyCode == ord('q'):
                    break
        finally:
            video_capture_0.release()
            video_capture_1.release()
            cv2.destroyAllWindows()
    else:
        print("Error: Unable to open one or both cameras. Check sensor-id (0 and 1) and connections.")
        if video_capture_0.isOpened(): video_capture_0.release()
        if video_capture_1.isOpened(): video_capture_1.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    show_two_cameras_optimized()