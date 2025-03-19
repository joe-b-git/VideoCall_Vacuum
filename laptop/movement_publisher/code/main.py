import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
import time
from movement_pb2 import Movement  # Assuming you have movement_pb2.py generated from movement.proto
import sys
import math

def main():
    """
    Main function to initialize eCAL and publish movement messages.
    """
    # Initialize eCAL
    ecal_core.initialize(sys.argv, "MovementPublisher")

    # Create publisher
    publisher = ProtoPublisher("movement", Movement)

    # Main loop
    try:
        velocity = 0.5  # Initial velocity
        yaw_rate = 0.0  # Initial yaw rate
        angle = 0
        while ecal_core.ok():
            # Create a Movement message
            movement_msg = Movement()
            movement_msg.velocity = velocity
            movement_msg.yaw_rate = yaw_rate

            # Publish the message
            publisher.send(movement_msg)

            # Print what you published
            print(f"Published: Velocity = {velocity}, Yaw Rate = {yaw_rate}")
            
            # Update velocity and yaw rate for the next cycle
            
            angle = angle + 0.1 
            yaw_rate = math.sin(angle) *0.3 #make a sinus movement with yaw rate
            
            # Sleep to control the publishing rate
            time.sleep(0.1)  # Publish every 100ms

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Cleanup
        ecal_core.finalize()

if __name__ == "__main__":
    main()
