# Simplified Camera Client using ZeroMQ (ZMQ)
# This script uses the ZMQVideoReceiver class to handle stream reception,
# making the main loop look identical to a standard OpenCV VideoCapture client.

import cv2 as cv
import time
import numpy as np # Still useful if you use NumPy operations later
# --- Import the custom wrapper class ---
from zmq_video_client import ZMQVideoReceiver 
# ---------------------------------------

# --- Configuration ---
ZMQ_ADDRESS = "tcp://127.0.0.1:5555"
# ---------------------

def start_client():
    """
    Connects to the ZMQ Publisher using the ZMQVideoReceiver and
    displays the received frames.
    """
    # 1. Initialize the ZMQ Video Receiver (replaces context/socket setup)
    cap = ZMQVideoReceiver(address=ZMQ_ADDRESS)
    
    if not cap.isOpened():
        # The receiver will print an error, we just exit here
        return
        
    print(f"[INFO] Client is ready to receive frames.")

    cv.namedWindow("Shared Camera Feed", cv.WINDOW_AUTOSIZE)

    try:
        while True:
            # 2. Read the frame (replaces all raw ZMQ receiving/reconstruction)
            ret, frame = cap.read()
            
            if not ret or frame is None:
                # No frame available, wait briefly and continue
                time.sleep(0.001)
                continue
            
            # --- Frame Processing Starts Here ---

            cv.imshow("Shared Camera Feed", frame)

            # Exit on 'q' press or ESC (27)
            key = cv.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break

    except KeyboardInterrupt:
        print("\n[INFO] Client stopped by user.")
    except Exception as e:
        print(f"\n[ERROR] An unexpected error occurred: {e}")
    finally:
        # 3. Release resources (replaces socket/context cleanup)
        cap.release()
        cv.destroyAllWindows()


if __name__ == "__main__":
    start_client()