import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
import cv2
import time
import numpy as np
from proto_python.image_pb2 import ImageMessage

def on_image(topic_name, msg, time):
    # msg is already an ImageMessage object when using ProtoSubscriber
    # Convert received data to cv::Mat
    frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, msg.channels))
    
    #Check if color image
    # if msg.channels == 3:
    #     frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Show the frame
    cv2.imshow("Webcam Feed", frame)
    cv2.waitKey(1)  # Process GUI events

def main():
    # Initialize eCAL
    ecal_core.initialize([], "WebcamSubscriber")
    
    # Create subscriber
    subscriber = ProtoSubscriber("webcam_feed", ImageMessage)
    
    # Set callback
    subscriber.set_callback(on_image)
    
    # Create window
    cv2.namedWindow("Webcam Feed", cv2.WINDOW_AUTOSIZE)
    
    # Main loop
    try:
        while ecal_core.ok():
            # Sleep to prevent busy waiting
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        ecal_core.finalize()

if __name__ == "__main__":
    main()
