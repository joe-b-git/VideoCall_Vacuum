from enum import Enum
from pid_controller import PIDController
from utils import calculate_bottom_center
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
        self.last_known_bottom_center = None
        self.yaw_pid = PIDController(kp=0.25, ki=0.0000, kd=0.0001, setpoint=0.0, output_limits=(-100, 100), deadzone=80)
        self.velocity_pid = PIDController(kp=0.002, ki=0.00000, kd=0.00001, setpoint=0.0, output_limits=(-0.25, 0.25), deadzone=20)
        self.person_behind_start_time = None
        self.person_sideways_start_time = None
        self.person_away_start_time = None
        self.person_lost_start_time = None
        self.bumper_sensor_data = None
        self.person_lost_threshold = 2.0  # Time in seconds before switching to FIND_SOMEONE
        self.person_behind_threshold = 8.0
        self.person_sideways_threshold = 6.0
        self.person_away_threshold = 3.0
        self.person_away_turn_threshold = 1.0 # Time in seconds before turning when person is away

    def update(self, person_found, person_box, width, height, bumper_sensor_data):
        print(f"Current State: {self.state}") #debug
        print(f"Last known bottom x,y: {self.last_known_bottom_center}")
        print("-----------------------------------------")

        self.bumper_sensor_data = bumper_sensor_data

        current_time = time.time()

        target_x = width / 2
        target_y = height - 20

        if person_found:
            self.state = FollowerState.PERSON_IN_FRAME
            self.last_known_bottom_center = calculate_bottom_center(person_box)
            self.person_behind_start_time = None
            self.person_sideways_start_time = None
            self.person_away_start_time = None
            self.person_lost_start_time = None
        else:
            if self.person_lost_start_time is None:
                self.person_lost_start_time = current_time

        if self.state == FollowerState.PERSON_IN_FRAME:
            if person_found:               
                bottom_center_x, bottom_center_y = calculate_bottom_center(person_box)
                _, _, box_width, _ = person_box
                if box_width > 320: # Limit the box width to avoid excessive yaw corrections
                    box_width = 320
                yaw_error = target_x - bottom_center_x
                yaw_rate = self.yaw_pid.calculate(-yaw_error, current_time, deadzone=box_width/4)
                velocity_error = target_y - bottom_center_y
                velocity = self.velocity_pid.calculate(-velocity_error, current_time, deadzone=20)

                self.send_movement(velocity, yaw_rate)
            else:
                bottom_center_x, bottom_center_y = self.last_known_bottom_center
                if bottom_center_y > height - 10: #person is behind
                    if self.person_behind_start_time is None:
                        self.person_behind_start_time = current_time
                    self.state = FollowerState.PERSON_BEHIND
                elif bottom_center_x < 150 or bottom_center_x > width - 150: #person is too far sideways
                    if self.person_sideways_start_time is None:
                        self.person_sideways_start_time = current_time
                    self.state = FollowerState.PERSON_SIDEWAYS
                elif bottom_center_y < 100: #person is walking away
                    if self.person_away_start_time is None:
                        self.person_away_start_time = current_time
                    self.state = FollowerState.PERSON_AWAY
                elif self.person_lost_start_time is not None and current_time - self.person_lost_start_time >= self.person_lost_threshold:
                    self.state = FollowerState.FIND_SOMEONE

        elif self.state == FollowerState.PERSON_BEHIND:
            if person_found:
                self.state = FollowerState.PERSON_IN_FRAME
                self.person_behind_start_time = None
            elif self.person_behind_start_time is not None and current_time - self.person_behind_start_time >= self.person_behind_threshold:
                self.state = FollowerState.FIND_SOMEONE
                self.person_behind_start_time = None
            else:
                # Turn 360 degrees in the direction of the last known person
                if self.last_known_bottom_center[0] < width / 2:
                    self.send_movement(0, 40)  # Turn left
                else:
                    self.send_movement(0, -40)  # Turn right

        elif self.state == FollowerState.PERSON_SIDEWAYS:
            if person_found:
                self.state = FollowerState.PERSON_IN_FRAME
                self.person_sideways_start_time = None
            elif self.person_sideways_start_time is not None and current_time - self.person_sideways_start_time >= self.person_sideways_threshold:
                self.state = FollowerState.FIND_SOMEONE
                self.person_sideways_start_time = None
            else:
                # Turn up to 180 degrees in the direction of the last known person
                if self.last_known_bottom_center[0] < width / 2:
                    self.send_movement(0, 30)  # Turn left
                else:
                    self.send_movement(0, -30)  # Turn right

        elif self.state == FollowerState.PERSON_AWAY:
            if person_found:
                self.state = FollowerState.PERSON_IN_FRAME
                self.person_away_start_time = None
            elif self.person_away_start_time is not None and current_time - self.person_away_start_time >= self.person_away_threshold:
                if self.last_known_bottom_center[0] < width / 2:
                    self.send_movement(0, 45)  # Turn left
                else:
                    self.send_movement(0, -45)  # Turn right
            elif self.person_away_start_time is not None and current_time - self.person_away_start_time >= self.person_away_threshold + self.person_away_turn_threshold:
                self.state = FollowerState.FIND_SOMEONE
                self.person_away_start_time = None
            else:
                # Move forward
                self.send_movement(0.1, 0)

        elif self.state == FollowerState.FIND_SOMEONE:
            if person_found:
                self.state = FollowerState.PERSON_IN_FRAME
                self.person_lost_start_time = None
            else:
                # Wall following logic
                if bumper_sensor_data["front_left"] or bumper_sensor_data["front_right"] or \
                        bumper_sensor_data["front_center_left"] or bumper_sensor_data["front_center_right"] or \
                        bumper_sensor_data["bump_left"] or bumper_sensor_data["bump_right"]:
                    self.send_movement(0, 30)  # Turn left away from wall
                elif bumper_sensor_data["right"]:
                    self.send_movement(0.1, 0)  # Move forward
                else:
                    self.send_movement(0.1, -5) #turn right slightly to find wall

    def send_movement(self, velocity, yaw_rate):
        from proto_python.movement_pb2 import Movement
        movement_message = Movement()
        if self.bumper_sensor_data["bump_left"] or self.bumper_sensor_data["bump_right"]:
            velocity = 0.0
        movement_message.velocity = velocity
        movement_message.yaw_rate = yaw_rate
        self.movement_publisher.send(movement_message)
