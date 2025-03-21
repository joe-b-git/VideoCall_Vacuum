import ecal.core.core as ecal_core
from ecal.core.publisher import ProtoPublisher
import time
import sys
from proto.sensors_pb2 import Sensors
from pyroombaadapter import PyRoombaAdapter

def main():
    # Initialize eCAL
    ecal_core.initialize(sys.argv, "SensorsPublisher")
    
    # Create publisher
    publisher = ProtoPublisher("sensors", Sensors)

    PORT = "/dev/ttyUSB0"
    adapter = PyRoombaAdapter(PORT)

    try:
        while ecal_core.ok():

            light_bumper_byte = adapter._request_sensor("Light Bumper")
            bumper_byte = adapter._request_sensor("Wheel Drops")
            wall_byte = adapter._request_sensor("Wall")

            sen_msg = Sensors()

            # Access the nested messages and set their fields
            sen_msg.light_bumper.light_bump_left = (light_bumper_byte >> 0) & 1 == 1
            sen_msg.light_bumper.light_bump_front_left = (light_bumper_byte >> 1) & 1 == 1
            sen_msg.light_bumper.light_bump_front_center_left = (light_bumper_byte >> 2) & 1 == 1
            sen_msg.light_bumper.light_bump_front_center_right = (light_bumper_byte >> 3) & 1 == 1
            sen_msg.light_bumper.light_bump_front_right = (light_bumper_byte >> 4) & 1 == 1
            sen_msg.light_bumper.light_bump_right = (light_bumper_byte >> 5) & 1 == 1

            sen_msg.bumper.bump_right = (bumper_byte >> 0) & 1 == 1
            sen_msg.bumper.bump_left = (bumper_byte >> 1) & 1 == 1

            sen_msg.wall.wall_seen = (wall_byte >> 0) & 1 == 1

            # Publish the message
            publisher.send(sen_msg)

            # Small delay to control publishing rate (adjust as needed)
            time.sleep(0.1)  # ~10 FPS

    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Cleanup
        ecal_core.finalize()

if __name__ == "__main__":
    main()
