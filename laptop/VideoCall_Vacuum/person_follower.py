from enum import Enum
from .pid_controller import PIDController
from .utils import calculate_bottom_center
import time

class FollowerState(Enum):
    PERSON_IN_FRAME = 1
    PERSON_BEHIND = 2
    PERSON_SIDEWAYS = 3
    PERSON_AWAY = 4
    FIND_SOMEONE = 5

class PersonFollower:
    def __init__(self, movement_publisher):
        self.movement_publisher = movement_publisher
        self.state = FollowerState.FIND_SOMEONE
        self.last_known_person_position = None
        self.last_known_bottom_center = None
        self.yaw_pid = PIDController(kp=0.005, ki=0.0001, kd=0.0001, setpoint=0.0, output_limits=(-120, 120), deadzone=10)
        self.velocity_pid = PIDController(kp=0.001, ki=0.00001, kd=0.00001, setpoint=0.0, output_limits=(-0.2, 0.2), deadzone=10)
        self.last_time = time.time()

    def update(self, person_found, person_box, person_position, width, bumper_data):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if person_found:
            self.state = FollowerState.PERSON_IN_FRAME
            self.last_known_person_position = person_position
            self.last_known_bottom_center = calculate_bottom_center(person_box)

        if self.state == FollowerState.PERSON_IN_FRAME:
            if person_found:
                center_x, center_y = person_position
                center_width = width / 2
                yaw_error = center_width - center_x
                yaw_rate = self.yaw_pid.calculate(yaw_error, current_time)
                
                bottom_center_x, bottom_center_y = calculate_bottom_center(person_box)
                velocity_error = 100 - bottom_center_y
                velocity = self.velocity_pid.calculate(velocity_error, current_time)

                self.send_movement(velocity, yaw_rate)

                if bottom_center_y > 400: #person is behind
                    self.state = FollowerState.PERSON_BEHIND
                elif center_x < 100 or center_x > width - 100: #person is too far sideways
                    self.state = FollowerState.PERSON_SIDEWAYS
                elif bottom_center_y < 100: #person is walking away
                    self.state = FollowerState.PERSON_AWAY
            else:
                self.state = FollowerState.FIND_SOMEONE

        elif self.state == FollowerState.PERSON_BEHIND:
            if person_found:
                self.state = FollowerState.PERSON_IN_FRAME
            else:
                # Turn 360 degrees in the direction of the last known person
                if self.last_known_bottom_center[0] < width / 2:
                    self.send_movement(0, 120)  # Turn left
                else:
                    self.send_movement(0, -120)  # Turn right
                time.sleep(3)
                self.state = FollowerState.FIND_SOMEONE

        elif self.state == FollowerState.PERSON_SIDEWAYS:
            if person_found:
                self.state = FollowerState.PERSON_IN_FRAME
            else:
                # Turn up to 90 degrees in the direction of the last known person
                if self.last_known_person_position[0] < width / 2:
                    self.send_movement(0, 90)  # Turn left
                else:
                    self.send_movement(0, -90)  # Turn right
                time.sleep(1.5)
                self.state = FollowerState.FIND_SOMEONE

        elif self.state == FollowerState.PERSON_AWAY:
            if person_found:
                self.state = FollowerState.PERSON_IN_FRAME
            else:
                # Move forward and turn up to 120 degrees
                if self.last_known_bottom_center[1] < 200:
                    self.send_movement(0.2, 0)  # Move forward
                else:
                    self.send_movement(0.1, 0)
                time.sleep(1)
                if self.last_known_person_position[0] < width / 2:
                    self.send_movement(0, 120)  # Turn left
                else:
                    self.send_movement(0, -120)  # Turn right
                time.sleep(2)
                self.state = FollowerState.FIND_SOMEONE

        elif self.state == FollowerState.FIND_SOMEONE:
            if person_found:
                self.state = FollowerState.PERSON_IN_FRAME
            else:
                # Wall following logic
                if bumper_data["front_left"] or bumper_data["front_right"]:
                    self.send_movement(0, -90)  # Turn right
                elif bumper_data["right"]:
                    self.send_movement(0.1, 0)  # Move forward
                else:
                    self.send_movement(0.1, -30) #turn right slightly
                time.sleep(0.1)

    def send_movement(self, velocity, yaw_rate):
        from proto_python.movement_pb2 import Movement
        movement_message = Movement()
        movement_message.velocity = velocity
        movement_message.yaw_rate = yaw_rate
        self.movement_publisher.send(movement_message)
