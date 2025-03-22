import cv2
import numpy as np
import os
from utils import calculate_center, calculate_bottom_center

class PersonDetector:
    def __init__(self, use_gpu=False):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.join(current_dir, 'data')

        if not os.path.exists(data_dir):
            raise FileNotFoundError(f"Data directory not found: {data_dir}")

        if use_gpu and cv2.cuda.getCudaEnabledDeviceCount() > 0:
            yolo_weights_path = os.path.join(data_dir, 'yolov4.weights')
            yolo_config_path = os.path.join(data_dir, 'yolov4.cfg')
        else:
            yolo_weights_path = os.path.join(data_dir, 'yolov4-tiny.weights')
            yolo_config_path = os.path.join(data_dir, 'yolov4-tiny.cfg')

        if not os.path.exists(yolo_weights_path):
            raise FileNotFoundError(f"YOLO weights file not found: {yolo_weights_path}")
        if not os.path.exists(yolo_config_path):
            raise FileNotFoundError(f"YOLO config file not found: {yolo_config_path}")

        self.net = cv2.dnn.readNet(yolo_weights_path, yolo_config_path)
        if use_gpu and cv2.cuda.getCudaEnabledDeviceCount() > 0:
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

        self.person_class_id = 0
        self.confidence_threshold = 0.5
        self.nms_threshold = 0.4
        self.prev_detections = None
        self.frame_count = 0
        self.last_known_person_position = None
        self.person_found = False

    def detect_people(self, img):
        height, width, _ = img.shape

        if self.frame_count % 2 == 0 or True:  # Run detection on every other frame
            blob = cv2.dnn.blobFromImage(img, 0.00392, (320, 320), (0, 0, 0), True, crop=False)
            self.net.setInput(blob)
            outs = self.net.forward(self.output_layers)
            self.prev_detections = outs
        elif self.prev_detections is not None:
            outs = self.prev_detections
        else:
            return False, None, width, height

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

        people_data = []  # Store (box, position) tuples
        self.person_found = False
        for i in range(len(boxes)):
            if i in indexes:
                if class_ids[i] == self.person_class_id:
                    self.person_found = True
                    x, y, w, h = boxes[i]
                    center_x, center_y = calculate_center((x, y, w, h))
                    people_data.append(((x, y, w, h), (center_x, center_y)))

        if self.person_found:
            # Find the most centered person in a single loop
            most_centered_person_box = None
            most_centered_person_position = None
            closest_person_distance = float('inf')
            center_width = width / 2
            for box, position in people_data:
                center_x, _ = position
                distance_to_center = abs(center_width - center_x)
                if distance_to_center < closest_person_distance:
                    closest_person_distance = distance_to_center
                    most_centered_person_box = box
                    most_centered_person_position = position
            self.last_known_person_position = most_centered_person_position
            return True, most_centered_person_box, width, height
        else:
            return False, None, width, height