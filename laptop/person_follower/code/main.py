import ecal.core.core as ecal_core
from ecal.core.subscriber import ProtoSubscriber
from ecal.core.publisher import ProtoPublisher
import cv2
import time
import numpy as np
from proto_python.image_pb2 import ImageMessage
from proto_python.movement_pb2 import Movement  # Import the Movement message
import os

class YoloDetector:
    def __init__(self):
        # Get the Yolo data, must download to the "data" folder
        current_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.join(current_dir, 'data')  # Correctly build the data directory path

        if not os.path.exists(data_dir):
            raise FileNotFoundError(f"Data directory not found: {data_dir}")

        if (cv2.cuda.getCudaEnabledDeviceCount() > 0):
            yolo_weights_path = os.path.join(data_dir, 'yolov4.weights')
            yolo_config_path = os.path.join(data_dir, 'yolov4.cfg')
        else:
            yolo_weights_path = os.path.join(data_dir, 'yolov4-tiny.weights')
            yolo_config_path = os.path.join(data_dir, 'yolov4-tiny.cfg')

        #check if the files are where they are expected to be.
        if not os.path.exists(yolo_weights_path):
            raise FileNotFoundError(f"YOLO weights file not found: {yolo_weights_path}")
        if not os.path.exists(yolo_config_path):
            raise FileNotFoundError(f"YOLO config file not found: {yolo_config_path}")

        # Load YOLO
        try:
            self.net = cv2.dnn.readNet(yolo_weights_path, yolo_config_path)
        except Exception as e:
            raise RuntimeError(f"Error loading YOLO model: {e}") from e

        if (cv2.cuda.getCudaEnabledDeviceCount() > 0):
            self.net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
            self.net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
        self.layer_names = self.net.getLayerNames()
        output_layers_indices = self.net.getUnconnectedOutLayers()
        if output_layers_indices.ndim == 1:
            # This is a 1-D numpy array of integers
            self.output_layers = [self.layer_names[i - 1] for i in output_layers_indices]
        else:
            # This is a 2-D numpy array of integers
            self.output_layers = [self.layer_names[i[0] - 1] for i in output_layers_indices]

        self.person_class_id = 0  # Assuming the class ID for 'person' is 0
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4
        self.prev_detections = None
        self.frame_count = 0

    def detect_people(self, img):
        height, width, _ = img.shape

        if self.frame_count % 2 == 0 or True:  # Run detection on every 5th frame
            blob = cv2.dnn.blobFromImage(img, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
            self.net.setInput(blob)
            outs = self.net.forward(self.output_layers)
            self.prev_detections = outs
        else:
            outs = self.prev_detections

        self.frame_count += 1

        class_ids = []
        confidences = []
        boxes = []
        for out in outs:
            for detection in out:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > self.confidence_threshold:
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        indexes = cv2.dnn.NMSBoxes(boxes, confidences, self.confidence_threshold, self.nms_threshold)

        people_boxes = []
        people_positions = []
        for i in range(len(boxes)):
            if i in indexes:
                if class_ids[i] == self.person_class_id:
                    x, y, w, h = boxes[i]
                    people_boxes.append((x, y, w, h))
                    top_center_x = x + w / 2  # Calculate top center x
                    top_center_y = y  # Calculate top center y
                    people_positions.append((top_center_x, top_center_y))

        return people_boxes, people_positions, width

def on_image(topic_name, msg, time, detector, movement_publisher):
    # msg is already an ImageMessage object when using ProtoSubscriber
    # Convert received data to cv::Mat and create a copy
    original_frame = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, msg.channels))
    frame = original_frame.copy() #this is the fix, we copy the array to make it writeable

    # Detect people
    people_boxes, people_positions, width = detector.detect_people(frame)
    center_width = width/2

    # Draw bounding boxes and find the most centered person
    most_centered_person_position = None
    closest_person_distance = float('inf')
    for (x, y, w, h), (top_center_x, top_center_y) in zip(people_boxes, people_positions): #get the top center data.
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 2)
        distance_to_center = abs(center_width - top_center_x) #check from the top center position.
        if distance_to_center < closest_person_distance:
            closest_person_distance = distance_to_center
            most_centered_person_position = (top_center_x, top_center_y) #report the top center position.

    # Movement logic
    yaw_rate = 0.0  # Default yaw rate
    velocity = 0.0  # Default velocity

    if most_centered_person_position:
        top_center_x = most_centered_person_position[0]
        
        if top_center_x < center_width - 200:
            yaw_rate = 120 
        elif top_center_x < center_width - 160:
            yaw_rate = 90
        elif top_center_x < center_width - 120:
            yaw_rate = 60        
        elif top_center_x < center_width - 80:
            yaw_rate = 30
        elif top_center_x > center_width + 80:
            yaw_rate = -30
        elif top_center_x > center_width + 120:
            yaw_rate = -60
        elif top_center_x > center_width + 160:
            yaw_rate = -90
        elif top_center_x > center_width + 200:
            yaw_rate = -120

        top_center_y = most_centered_person_position[1]

        if top_center_y < 10:
            velocity = 0
        elif top_center_y > 110:
            velocity = 0.2
        else:
            velocity = 0.0
    
    #check if values have changed
    if yaw_rate != on_image.prev_yaw_rate or velocity != on_image.prev_velocity:
        movement_message = Movement()
        movement_message.velocity = velocity
        movement_message.yaw_rate = yaw_rate
        movement_publisher.send(movement_message)
        on_image.prev_yaw_rate = yaw_rate
        on_image.prev_velocity = velocity

    #report results.
    #if most_centered_person_position:
    #    print(f"Most centered person position: {most_centered_person_position}")

    # Show the frame
    cv2.imshow("Webcam Feed", frame)
    cv2.waitKey(1)  # Process GUI events

# Initialize previous yaw rate
on_image.prev_yaw_rate = None
on_image.prev_velocity = None

def main():
    # Initialize eCAL
    ecal_core.initialize([], "PersonFollower")

    # Create subscriber
    subscriber = ProtoSubscriber("webcam_feed", ImageMessage)

    # Create movement publisher
    movement_publisher = ProtoPublisher("movement_command", Movement)

    # Initialize YOLO detector
    try:
        detector = YoloDetector()
    except (FileNotFoundError, RuntimeError) as e:
        print(f"Error initializing YoloDetector: {e}")
        return  # Exit if detector initialization fails

    # Set callback
    subscriber.set_callback(lambda topic, msg, time: on_image(topic, msg, time, detector, movement_publisher))

    # Create window
    cv2.namedWindow("Webcam Feed", cv2.WINDOW_AUTOSIZE)

    # Main loop
    try:
        while ecal_core.ok():
            # Sleep to prevent busy waiting
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        # Cleanup
        cv2.destroyAllWindows()
        ecal_core.finalize()

if __name__ == "__main__":
    main()
