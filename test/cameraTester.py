# ZMQ Frame Rate Tester
# Connects to the cameraPublisher.py stream and calculates the actual FPS.

import time
import sys
import numpy as np

# Import the necessary class from your existing library
try:
    from zmq_video_client import ZMQVideoReceiver
except ImportError:
    print("[ERROR] Could not import ZMQVideoReceiver. Make sure 'zmq_video_client.py' is in the same directory.")
    sys.exit(1)

# Configuration
ZMQ_ADDRESS = "tcp://127.0.0.1:5555"
REPORT_INTERVAL = 1.0 # Report FPS every second

def test_fps():
    """Connects to the ZMQ stream and calculates the frame rate."""
    cap = ZMQVideoReceiver(address=ZMQ_ADDRESS)

    if not cap.isOpened():
        print(f"Failed to connect to ZMQ stream at {ZMQ_ADDRESS}.")
        return

    print(f"\n[FPS Tester] Successfully subscribed to {ZMQ_ADDRESS}. Measuring FPS...")
    print(f"[FPS Tester] Aiming for {15} FPS (1 frame every ~66.7ms)")
    
    frame_count = 0
    start_time = time.time()
    last_report_time = start_time

    try:
        while True:
            # Non-blocking read from the camera publisher
            ret, frame = cap.read()
            
            if ret:
                frame_count += 1

                current_time = time.time()

                # Check if it's time to report the FPS
                if current_time - last_report_time >= REPORT_INTERVAL:
                    elapsed_time = current_time - last_report_time
                    
                    if elapsed_time > 0:
                        # Calculate instantaneous FPS for the past interval
                        current_fps = frame_count / elapsed_time
                        
                        print(
                            f"[FPS Log] Frames: {frame_count} | Interval FPS: {current_fps:.2f}"
                        )

                    # Reset counters
                    frame_count = 0
                    last_report_time = current_time
            else:
                # Sleep briefly if no frame was available
                time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[FPS Tester] Stopped by user.")
    except Exception as e:
        print(f"\n[FATAL ERROR] An unexpected error occurred: {e}")
    finally:
        cap.release()
        print("[FPS Tester] Resources released.")

if __name__ == "__main__":
    test_fps()