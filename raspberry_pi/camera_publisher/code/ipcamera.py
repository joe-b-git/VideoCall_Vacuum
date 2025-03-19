import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
import time
import cv2
import numpy as np
from proto_python.image_pb2 import ImageMessage
import socket
import threading

class IPCameraRelay:
    def __init__(self, host, port, ecal_topic="webcam_feed", frame_width=640, frame_height=480):
        self.host = host
        self.port = port
        self.ecal_topic = ecal_topic
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.server_socket = None
        self.client_socket = None
        self.is_running = False
        self.latest_frame = None
        self.latest_frame_lock = threading.Lock()

        # Initialize eCAL
        ecal_core.initialize(sys.argv, "IPCameraRelay")

        # Create subscriber
        self.subscriber = ProtoSubscriber(self.ecal_topic, ImageMessage)
        self.subscriber.set_callback(self.ecal_callback)

    def ecal_callback(self, topic_name, msg, time):
        # Receive frame from eCAL and store it
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, msg.channels))

        with self.latest_frame_lock:
            self.latest_frame = frame

    def start_server(self):
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen()
        print(f"IP Camera Relay Server started on {self.host}:{self.port}")

        self.is_running = True
        threading.Thread(target=self.accept_connections, daemon=True).start()

        try:
            while ecal_core.ok() and self.is_running:
                self.relay_frame()
                time.sleep(0.01)

        except KeyboardInterrupt:
            print("Shutting down...")
        finally:
            self.stop_server()

    def accept_connections(self):
        while self.is_running:
            try:
                self.client_socket, addr = self.server_socket.accept()
                print(f"Connection from {addr}")
                # Handle the client in a separate thread or process if needed
            except Exception as e:
                print(f"Error accepting connection: {e}")
                break

    def relay_frame(self):
        with self.latest_frame_lock:
            frame = self.latest_frame
            
        if frame is None:
            return  # No frame yet

        if self.client_socket:
            try:
                encoded, buffer = cv2.imencode('.jpg', frame)
                jpg_as_text = buffer.tobytes()
                self.client_socket.sendall(jpg_as_text)

            except Exception as e:
                print(f"Error sending frame: {e}")
                self.client_socket.close()
                self.client_socket = None

    def stop_server(self):
        self.is_running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        ecal_core.finalize()
        print("IP Camera Relay Server stopped.")

if __name__ == "__main__":
    import sys
    
    # Change the IP and port as needed
    host = "192.168.0.100"  # Listen on all interfaces
    port = 8080

    ip_camera_relay = IPCameraRelay(host, port)
    ip_camera_relay.start_server()
