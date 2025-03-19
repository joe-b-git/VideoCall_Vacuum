import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
import time
import cv2
import numpy as np
from proto_python.image_pb2 import ImageMessage

def main():
    # Initialize eCAL
    ecal_core.initialize(sys.argv, "WebcamPublisher")
    
    # Create publisher
    publisher = ProtoPublisher("webcam_feed", ImageMessage)

    # Open the webcam (0 is usually the default)
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    # Set webcam properties (resolution)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 360)

    try:
        while ecal_core.ok():
            # Capture a frame
            ret, frame = cap.read()
            if not ret:
                print("Error: Could not read frame.")
                break

            # Convert frame to message
            img_msg = ImageMessage()
            img_msg.timestamp = int(time.time())
            img_msg.width = frame.shape[1]
            img_msg.height = frame.shape[0]
            img_msg.channels = frame.shape[2] if len(frame.shape) == 3 else 1  # Handle grayscale or color
            img_msg.data = frame.tobytes()

            # Publish the message
            publisher.send(img_msg)

            # Small delay to control publishing rate (adjust as needed)
            # time.sleep(0.1)  # ~10 FPS

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Cleanup
        cap.release()
        ecal_core.finalize()

if __name__ == "__main__":
    import sys
    main()
