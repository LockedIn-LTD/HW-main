# Camera Driver using ZeroMQ (ZMQ) for Inter-Process Communication (IPC)
# This script runs once and streams the camera feed to multiple subscribers.

import time
import cv2 as cv
import numpy as np
import zmq
import sys
import argparse 

# --- Configuration (Defaults/Constants) ---
DEFAULT_WIDTH = 1280
DEFAULT_HEIGHT = 720
DEFAULT_FPS = 15 # Set desired FPS here
# ---------------------

# IMX219 modes (Argus):
# 0: 3280x2464 @21, 1: 3280x1848 @28, 2: 1920x1080 @30, 3: 1640x1232 @30, 4: 1280x720 @60


def gst_pipeline(sensor_id, width, height, fps=DEFAULT_FPS, flip=0):
    """Generates the GStreamer pipeline string for nvarguscamerasrc."""

    # Note: nvarguscamerasrc will likely run at 60 FPS for 1280x720,
    # so we must explicitly cap the rate using the 'videorate' element.
    print(f"Applying framerate constraint to {fps} FPS using videorate element.")
    
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} bufapi-version=1 ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, format=NV12, framerate={fps}/1 ! " # Use native Argus framerate limit
        
        # 2. Convert/Flip/Scale in Hardware (NVMM to NVMM BGRx)
        f"nvvidconv flip-method={flip} ! "
        f"video/x-raw(memory:NVMM), format=BGRx ! "

        # 3. Convert from NVMM (hardware) to CPU memory (This is the critical step)
        f"nvvidconv ! " # No arguments needed, just for NVMM -> CPU transition
        f"video/x-raw, format=BGRx ! "
        
        # 4. Convert BGRx (4-channel) to BGR (3-channel) in CPU (if necessary)
        f"videoconvert ! "
        f"video/x-raw, format=BGR, width={width}, height={height} ! "
        
        # Optional: Use videorate if frame dropping is still required
        # Note: Setting framerate in step 1 is often enough.
        f"appsink drop=true max-buffers=1 sync=false")


def open_nvargus(sensor_id, width, height, fps=DEFAULT_FPS, flip=0):
    """Opens the GStreamer camera capture object and performs a warmup."""
    
    pipeline = gst_pipeline(sensor_id, width, height,
                            fps=fps, flip=flip)
    
    
    print(
        f"[INFO] Opening Argus sensor-id={sensor_id} {width}x{height}@{fps} flip={flip}")
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
        cap = open_nvargus(camera_id, width, height, fps=DEFAULT_FPS, flip=flip) 
    except RuntimeError as e:
        print(f"[ERROR] {e}")
        sys.exit(1)

    # Main frame loop
    try:
        while True:
            # This read will now be naturally limited to 15 FPS by the videorate element
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
        '--flip', type=int, default=0, 
        help=f'Flip method for nvvidconv (0=none, 1=horizontal, 2=rotate 180, 3=vertical, 4=horiz+diag, 5=horiz+vert, 6=vert+diag, 7=rotate 90, 8=rotate 270) (default: 2).'
    )
    
    args = parser.parse_args()
    return args.id, str(args.port), args.width, args.height, args.flip


if __name__ == "__main__":
    camera_id, zmq_port, width, height, flip = parse_args()
    start_driver(camera_id, zmq_port, width, height, flip)