import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import cv2
import time
import numpy as np
from proto_python.image_pb2 import ImageMessage
from proto_python.movement_pb2 import Movement
import threading
import pygame

class ManualControl:
    def __init__(self, movement_publisher):
        self.yaw_rate = 0.0
        self.velocity = 0.0
        self.movement_publisher = movement_publisher
        self.last_movement_update_time = time.time()
        self.movement_update_interval = 0.05  # Only update every 50ms
        self.is_running = True
        self.joystick = None
        self.initialize_controller()  # init the controller before starting the thread
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()
        self.event_thread = threading.Thread(target=self.event_loop, daemon=True) #create a thread for the event loop.
        self.event_thread.start()

    def initialize_controller(self):
        pygame.init()
        pygame.joystick.init()

        # Enumerate joysticks and print their names
        joystick_count = pygame.joystick.get_count()
        print(f"Number of joysticks: {joystick_count}")
        for i in range(joystick_count):
            joystick_name = pygame.joystick.Joystick(i).get_name()
            print(f"Joystick {i}: {joystick_name}")

        if joystick_count > 0:
            for i in range(joystick_count):
                if "Sony" in pygame.joystick.Joystick(i).get_name(): #check if it has sony in the name.
                    self.joystick = pygame.joystick.Joystick(i)
                    break

            if not self.joystick: #if it was not found, use the first one.
                self.joystick = pygame.joystick.Joystick(0)

            self.joystick.init()
            print(f"Controller '{self.joystick.get_name()}' connected.")
        else:
            print("No controller detected.")

    def event_loop(self):
        while self.is_running:
            pygame.event.pump()  # Process controller events constantly.

    def run(self):
        while self.is_running:
            self.update_movement()
            time.sleep(0.01)  # Reduced sleep time

    def update_movement(self):
        new_yaw_rate = 0.0
        new_velocity = 0.0

        if self.joystick:  # check that there is a controller before doing anything else.
            # Get the values of the analog sticks
            left_stick_y = self.joystick.get_axis(1)  # Left stick up/down (axis 1)
            right_stick_x = self.joystick.get_axis(3)  # Right stick left/right (axis 2)

            # Apply deadzone to left stick Y (velocity)
            if abs(left_stick_y) < 0.1:
                left_stick_y = 0.0
         
            # Apply deadzone to right stick X (yaw rate)
            if abs(right_stick_x) < 0.1:
                right_stick_x = 0.0

            # Set the velocity based on the left stick's Y axis, using the full range.
            new_velocity = -left_stick_y * 1.5  # invert the axis so up is positive.

            # Set the yaw rate based on the right stick's X axis, using the full range.
            new_yaw_rate = right_stick_x * -135.0  # multiply by 360, and invert the axis.

            # Print the values for debugging
            print(f"Left Y: {left_stick_y:.2f}, Right X: {right_stick_x:.2f} | new_velocity: {new_velocity:.2f} new_yaw_rate:{new_yaw_rate:.2f}")

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
        self.event_thread.join()
        if self.joystick:
            pygame.joystick.quit()
        pygame.quit()

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
        self.frame = frame.copy()  # copy the frame

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
        video_display.stop()  # stop the display thread
        ecal_core.finalize()

if __name__ == "__main__":
    main()
