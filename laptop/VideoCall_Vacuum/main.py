import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import cv2
import time
import numpy as np
from proto_python.image_pb2 import ImageMessage
from proto_python.light_bumper_pb2 import LightBumper  # Import the LightBumper message
from proto_python.movement_pb2 import Movement
from person_detector import PersonDetector
from person_follower import PersonFollower

class VideoCallVacuum:
    def __init__(self):
        ecal_core.initialize([], "VideoCallVacuum")
        self.image_subscriber = ProtoSubscriber("webcam_feed", ImageMessage)
        self.bumper_subscriber = ProtoSubscriber("light_bumper", LightBumper)
        self.movement_publisher = ProtoPublisher("movement_command", Movement)
        self.detector = PersonDetector(use_gpu=True)
        self.follower = PersonFollower(self.movement_publisher)
        self.bumper_data = {"front_left": False, "front_right": False, "right": False, "left": False, "front_center_left": False, "front_center_right": False}

        self.image_subscriber.set_callback(self.on_image)
        self.bumper_subscriber.set_callback(self.on_bumper)

        cv2.namedWindow("Webcam Feed", cv2.WINDOW_AUTOSIZE)

    def on_image(self, topic_name, msg, time):
        # msg is already an ImageMessage object when using ProtoSubscriber
        # Convert received data to cv::Mat and create a copy
        original_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, msg.channels))
        frame = original_frame.copy() #this is the fix, we copy the array to make it writeable

        person_found, person_box, person_position, width = self.detector.detect_people(frame)

        if person_found:
            x, y, w, h = person_box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)

        self.follower.update(person_found, person_box, person_position, width, self.bumper_data)

        cv2.imshow("Webcam Feed", frame)
        cv2.waitKey(1)

    def on_bumper(self, topic_name, msg, time):
        self.bumper_data["front_left"] = msg.light_bump_front_left
        self.bumper_data["front_right"] = msg.light_bump_front_right
        self.bumper_data["right"] = msg.light_bump_right
        self.bumper_data["left"] = msg.light_bump_left
        self.bumper_data["front_center_left"] = msg.light_bump_front_center_left
        self.bumper_data["front_center_right"] = msg.light_bump_front_center_right

    def run(self):
        try:
            while ecal_core.ok():
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            cv2.destroyAllWindows()
            ecal_core.finalize()

def main():
    vacuum = VideoCallVacuum()
    vacuum.run()

if __name__ == "__main__":
    main()
