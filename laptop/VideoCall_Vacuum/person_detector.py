import cv2
import numpy as np
import os
from utils import calculate_center, calculate_bottom_center

class KalmanFilter:
    def __init__(self, dt, u_x, u_y, std_acc, x_std_meas, y_std_meas):
        self.dt = dt
        self.u = np.matrix([[u_x], [u_y]])
        self.A = np.matrix([[1, 0, self.dt, 0],
                            [0, 1, 0, self.dt],
                            [0, 0, 1, 0],
                            [0, 0, 0, 1]])
        self.B = np.matrix([[(self.dt**2)/2, 0],
                            [0, (self.dt**2)/2],
                            [self.dt, 0],
                            [0, self.dt]])
        self.H = np.matrix([[1, 0, 0, 0],
                            [0, 1, 0, 0]])
        self.Q = np.matrix([[(self.dt**4)/4, 0, (self.dt**3)/2, 0],
                            [0, (self.dt**4)/4, 0, (self.dt**3)/2],
                            [(self.dt**3)/2, 0, self.dt**2, 0],
                            [0, (self.dt**3)/2, 0, self.dt**2]]) * std_acc**2
        self.R = np.matrix([[x_std_meas**2, 0],
                            [0, y_std_meas**2]])
        self.P = np.eye(self.A.shape[1])
        self.x = np.matrix([[0], [0], [0], [0]])

    def predict(self):
        self.x = np.dot(self.A, self.x) + np.dot(self.B, self.u)
        self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
        return self.x[0:2]

    def update(self, z):
        S = np.dot(self.H, np.dot(self.P, self.H.T)) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        y = z - np.dot(self.H, self.x)
        self.x = self.x + np.dot(K, y)
        I = np.eye(self.H.shape[1])
        self.P = (I - np.dot(K, self.H)) * self.P

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
        self.confidence_threshold = 0.6 # 0.5
        self.nms_threshold = 0.4
        self.prev_detections = None
        self.frame_count = 0
        self.last_known_person_position = None
        self.person_found = False
        self.kalman_filter = KalmanFilter(dt=0.1, u_x=0.1, u_y=0.1, std_acc=0.1, x_std_meas=0.1, y_std_meas=0.1)

        # Optical flow parameters
        self.lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))
        self.prev_gray = None
        self.prev_points = None

    def detect_people(self, img):
        height, width, _ = img.shape

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is not None and self.prev_points is not None:
            next_points, status, _ = cv2.calcOpticalFlowPyrLK(self.prev_gray, gray, self.prev_points, None, **self.lk_params)
            good_new = next_points[status.ravel() == 1]
            good_old = self.prev_points[status.ravel() == 1]

            for i, (new, old) in enumerate(zip(good_new, good_old)):
                a, b = new.ravel()
                c, d = old.ravel()
                cv2.line(img, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
                cv2.circle(img, (int(a), int(b)), 5, (0, 255, 0), -1)

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
            self.kalman_filter.update(np.matrix(most_centered_person_position).T)
            self.prev_gray = gray.copy()
            self.prev_points = np.array([most_centered_person_position], dtype=np.float32)
            return True, most_centered_person_box, width, height, None
        else:
            predicted_position = self.kalman_filter.predict()
            self.prev_gray = gray.copy()
            self.prev_points = np.array([predicted_position], dtype=np.float32)
            return False, None, width, height, predicted_position
