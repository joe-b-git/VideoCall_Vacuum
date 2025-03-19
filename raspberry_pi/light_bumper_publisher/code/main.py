import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
import time
import sys
from proto_python.light_bumper_pb2 import LightBumper
from pyroombaadapter import PyRoombaAdapter

def main():
    # Initialize eCAL
    ecal_core.initialize(sys.argv, "LightBumperPublisher")
    
    # Create publisher
    publisher = ProtoPublisher("light_bumper", LightBumper)

    PORT = "/dev/ttyUSB0"
    adapter = PyRoombaAdapter(PORT)

    try:
        while ecal_core.ok():

            light_bumper = adapter._request_sensor("Light Bumper")

            lb_msg = LightBumper()
            lb_msg.light_bump_left = (light_bumper >> 0) & 1 == 1
            lb_msg.light_bump_front_left = (light_bumper >> 1) & 1 == 1
            lb_msg.light_bump_front_center_left = (light_bumper >> 2) & 1 == 1
            lb_msg.light_bump_front_center_right = (light_bumper >> 3) & 1 == 1
            lb_msg.light_bump_front_right = (light_bumper >> 4) & 1 == 1
            lb_msg.light_bump_right = (light_bumper >> 5) & 1 == 1

            # Publish the message
            publisher.send(lb_msg)

            # Small delay to control publishing rate (adjust as needed)
            time.sleep(0.1)  # ~10 FPS

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Cleanup
        ecal_core.finalize()

if __name__ == "__main__":
    main()
