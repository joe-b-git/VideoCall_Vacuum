import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import cv2
import time
import numpy as np
from proto_python.image_pb2 import ImageMessage
from proto_python.movement_pb2 import Movement
import keyboard
import threading

class ManualControl:
    def __init__(self, movement_publisher):
        self.yaw_rate = 0.0
        self.velocity = 0.0
        self.movement_publisher = movement_publisher
        self.last_movement_update_time = time.time()
        self.movement_update_interval = 0.05  # Only update every 50ms
        self.is_running = True
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        while self.is_running:
            self.update_movement()
            time.sleep(0.01)  # Reduced sleep time

    def update_movement(self):
        new_yaw_rate = 0.0
        new_velocity = 0.0

        if keyboard.is_pressed('up'):
            new_velocity = 0.8
        elif keyboard.is_pressed('down'):
            new_velocity = -0.8

        if keyboard.is_pressed('left'):
            new_yaw_rate = 270.0
        elif keyboard.is_pressed('right'):
            new_yaw_rate = -270.0

        current_time = time.time()
        if (new_yaw_rate != self.yaw_rate or new_velocity != self.velocity) and current_time - self.last_movement_update_time >= self.movement_update_interval:
            self.yaw_rate = new_yaw_rate
            self.velocity = new_velocity
            self.last_movement_update_time = current_time
            self.send_movement()

    def send_movement(self):
        movement_message = Movement()
        movement_message.velocity = self.velocity
        movement_message.yaw_rate = self.yaw_rate
        self.movement_publisher.send(movement_message)

    def stop(self):
        self.is_running = False
        self.thread.join()

class VideoDisplay:
    def __init__(self):
        self.is_running = True
        self.frame = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        cv2.namedWindow("Webcam Feed", cv2.WINDOW_AUTOSIZE)
        while self.is_running:
            if self.frame is not None:
                cv2.imshow("Webcam Feed", self.frame)
            cv2.waitKey(1)

    def update_frame(self, frame):
        self.frame = frame.copy() #copy the frame
    
    def stop(self):
        self.is_running = False
        self.thread.join()
        cv2.destroyAllWindows()

def on_image(topic_name, msg, time, video_display):
    original_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, msg.channels))
    video_display.update_frame(original_frame)

def main():
    ecal_core.initialize([], "ManualControl")
    subscriber = ProtoSubscriber("webcam_feed", ImageMessage)
    movement_publisher = ProtoPublisher("movement_command", Movement)

    manual_control = ManualControl(movement_publisher)
    video_display = VideoDisplay()

    subscriber.set_callback(lambda topic, msg, time: on_image(topic, msg, time, video_display))

    try:
        while ecal_core.ok():
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        manual_control.stop()  # Stop the movement thread
        video_display.stop() #stop the display thread
        ecal_core.finalize()

if __name__ == "__main__":
    main()
