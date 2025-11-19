# Camera Driver using ZeroMQ (ZMQ) for Inter-Process Communication (IPC)
# This script runs once and streams the camera feed to multiple subscribers.

import time
import cv2 as cv
import numpy as np
import zmq
import sys
import argparse 

# --- Configuration (Defaults/Constants) ---
DEFAULT_WIDTH = 1920
DEFAULT_HEIGHT = 1080
DEFAULT_FLIP = 0
# ---------------------

# IMX219 modes (Argus):
# 0: 3280x2464 @21, 1: 3280x1848 @28, 2: 1920x1080 @30, 3: 1640x1232 @30, 4: 1280x720 @60


def get_sensor_mode_and_fps(width, height):
    """Maps desired resolution to the optimal IMX219 sensor mode and FPS."""
    mapping = {
        (3280, 2464): (0, 21),
        (3280, 1848): (1, 28),
        (1920, 1080): (2, 30),
        (1640, 1232): (3, 30),
        (1280, 720):  (4, 60),
    }
    
    # Check if the requested resolution exactly matches a known mode
    return mapping.get((int(width), int(height)), (2, 30)) # Default to 1920x1080 @30


def gst_pipeline(sensor_id, width, height, fps=None, flip=DEFAULT_FLIP, sensor_mode=None):
    """Generates the GStreamer pipeline string for nvarguscamerasrc."""
    
    # Use the requested width and height for the sensor mode lookup
    mode_auto, fps_auto = get_sensor_mode_and_fps(width, height) 
    
    sensor_mode = sensor_mode if sensor_mode is not None else mode_auto
    fps = fps if fps is not None else fps_auto

    # The pipeline outputs BGR format frames ready for OpenCV
    return (
        f"nvarguscamerasrc sensor-id={int(sensor_id)} sensor-mode={int(sensor_mode)} "
        f"bufapi-version=1 ! "
        f"video/x-raw(memory:NVMM), width=(int){int(width)}, height=(int){int(height)}, "
        f"framerate=(fraction){int(fps)}/1, format=(string)NV12 ! "
        f"nvvidconv flip-method={int(flip)} ! "
        f"video/x-raw, format=(string)BGRx, width=(int){int(width)}, height=(int){int(height)} ! "
        f"videoconvert ! "
        f"appsink caps=video/x-raw,format=(string)BGR,width=(int){int(width)},height=(int){int(height)} "
        f"drop=true max-buffers=1 sync=false"
    )


def open_nvargus(sensor_id, width, height, fps=None, flip=DEFAULT_FLIP, sensor_mode=None):
    """Opens the GStreamer camera capture object and performs a warmup."""
    
    pipeline = gst_pipeline(sensor_id, width, height,
                            fps=fps, flip=flip, sensor_mode=sensor_mode)
    
    mode_auto, fps_auto = get_sensor_mode_and_fps(width, height)
    
    print(
        f"[INFO] Opening Argus sensor-id={sensor_id} mode={sensor_mode or mode_auto} {width}x{height}@{fps or fps_auto} flip={flip}")
    cap = cv.VideoCapture(pipeline, cv.CAP_GSTREAMER)
    if not cap.isOpened():
        raise RuntimeError(
            f"Failed to open nvargus camera sensor-id={sensor_id}. Check hardware and permissions.")

    # Warmup frames to ensure the pipeline is stable and frames are being delivered
    print("[INFO] Warming up camera...")
    ok = False
    for _ in range(15):
        ret, frame = cap.read()
        if ret and frame is not None and frame.size > 0:
            ok = True
            break
        time.sleep(0.02)
    if not ok:
        cap.release()
        raise RuntimeError(
            f"Camera sensor-id={sensor_id} opened but no frames delivered after warmup.")
    print("[INFO] Camera ready. Starting ZMQ Publisher.")
    return cap


def start_driver(camera_id, zmq_port, width, height, flip):
    """Initializes the camera and ZMQ publisher to start streaming frames."""
    context = zmq.Context()
    socket = context.socket(zmq.PUB)
    
    # Use the port provided via the command line argument
    socket.bind(f"tcp://*:{zmq_port}")
    print(f"[INFO] ZMQ Publisher bound to port {zmq_port}")
    
    try:
        # Pass new arguments to the camera opening function
        cap = open_nvargus(camera_id, width, height, flip=flip) 
    except RuntimeError as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    # Main frame loop
    try:
        while True:
            ret, frame = cap.read()
            if not ret or frame is None:
                # If frame reading fails, wait and continue the loop
                time.sleep(0.01)
                continue

            # 1. Serialize Frame Metadata (Shape and Data Type)
            # We use the actual frame shape to ensure correct reconstruction on the client side.
            md = dict(
                dtype=str(frame.dtype),
                shape=frame.shape,
            )

            # 2. Send Metadata first using the DUMP method for object serialization
            socket.send_pyobj(md, flags=zmq.SNDMORE)

            # 3. Send the raw image data (the frame)
            socket.send(frame.tobytes())


    except KeyboardInterrupt:
        print("\n[INFO] Driver stopped by user.")
    except Exception as e:
        print(f"\n[FATAL ERROR] An unexpected error occurred: {e}")
    finally:
        # Cleanup resources
        cap.release()
        socket.close()
        context.term()


def parse_args():
    """Parses command line arguments for camera ID, ZMQ port, resolution, and flip method."""
    parser = argparse.ArgumentParser(description="Jetson Camera Driver streaming via ZMQ.")
    
    parser.add_argument(
        '--id', type=int, default=0, help='Camera sensor ID (default: 0). Use 0, 1, etc., for multiple cameras.'
    )
    parser.add_argument(
        '--port', type=int, default=5555, help='ZMQ port to bind the publisher to (default: 5555).'
    )
    # New argument for width
    parser.add_argument(
        '--width', type=int, default=DEFAULT_WIDTH, help=f'Desired camera capture width (default: {DEFAULT_WIDTH}).'
    )
    # Argument for height
    parser.add_argument(
        '--height', type=int, default=DEFAULT_HEIGHT, help=f'Desired camera capture height (default: {DEFAULT_HEIGHT}).'
    )
    # Argument for flip
    parser.add_argument(
        '--flip', type=int, default=DEFAULT_FLIP, 
        help=f'Flip method for nvvidconv (0=none, 1=horizontal, 2=rotate 180, 3=vertical, 4=horiz+diag, 5=horiz+vert, 6=vert+diag, 7=rotate 90, 8=rotate 270) (default: {DEFAULT_FLIP}).'
    )
    
    args = parser.parse_args()
    return args.id, str(args.port), args.width, args.height, args.flip


if __name__ == "__main__":
    camera_id, zmq_port, width, height, flip = parse_args()
    start_driver(camera_id, zmq_port, width, height, flip)