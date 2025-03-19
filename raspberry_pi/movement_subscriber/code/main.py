import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
import time
from movement_pb2 import Movement  # Assuming you have movement_pb2.py generated from movement.proto
import sys
import math
from pyroombaadapter import PyRoombaAdapter

PORT = "/dev/ttyUSB0"
adapter = PyRoombaAdapter(PORT)
adapter.change_mode_to_full()

def on_movement(topic_name, msg, time_):
    """
    Callback function to process received movement messages.

    Args:
        topic_name (str): The name of the topic the message was received from.
        msg (Movement): The received Movement message object.
        time_ (int): The timestamp of the received message.
    """
    try:
        velocity = msg.velocity
        yaw_rate = msg.yaw_rate
        print(f"Received Movement: Velocity = {velocity}, Yaw Rate = {yaw_rate}")

        adapter.move(velocity, math.radians(yaw_rate))

    except Exception as e:
        print(f"Error processing movement message: {e}")

def subscribe_to_movement():
    """
    Initializes eCAL and subscribes to the "movement" topic to receive
    velocity and yaw_rate messages.
    """
    # Initialize eCAL
    ecal_core.initialize(sys.argv, "MovementSubscriber")

    # Create subscriber
    subscriber = ProtoSubscriber("movement_command", Movement)

    # Set callback
    subscriber.set_callback(on_movement)

    # Main loop
    try:
        while ecal_core.ok():
            # Sleep to prevent busy waiting
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Cleanup
        ecal_core.finalize()

def main():
    subscribe_to_movement()

if __name__ == "__main__":
    main()
