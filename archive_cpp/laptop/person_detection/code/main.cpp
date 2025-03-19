#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ecal/ecal.h>
#include <ecal/msg/string/publisher.h>
#include <ecal/msg/string/subscriber.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <ecal/ecal_process.h>
#include <fstream>
#include <algorithm>

// Forward declaration of the struct used in the original code
namespace Yolov4Detector{
    struct Detection{
        int x;
        int y;
        int width;
        int height;
        int classId;
        float confidence;
    };
}


// Dummy implementation of Yolov4Detector class
class Yolov4Detector {
public:
    Yolov4Detector(const std::string& weightsPath, const std::string& cfgPath, const std::string& namesPath) {
        // In the real implementation, you would load the YOLO model here
        std::cout << "Yolov4Detector initialized (dummy)." << std::endl;
        isValid_ = true; // Assuming success in this dummy version
    }

    bool isValid() const {
        return isValid_;
    }

    std::vector<Yolov4Detector::Detection> detect(const cv::Mat& frame) {
        // In the real implementation, you would perform the YOLO inference here
        std::cout << "YOLOv4 detection called (dummy)." << std::endl;
        std::vector<Yolov4Detector::Detection> detections;
        // Simulate some detections (replace this with real detections later)
        
        if(frame.cols > 0){
          Yolov4Detector::Detection det1,det2, det3;

          det1.x = 50; det1.y = 50; det1.width = 100; det1.height = 200; det1.classId = 0; det1.confidence = 0.8;
          detections.push_back(det1);
          
          det2.x = 200; det2.y = 100; det2.width = 150; det2.height = 250; det2.classId = 0; det2.confidence = 0.9;
          detections.push_back(det2);

          det3.x = 400; det3.y = 10; det3.width = 100; det3.height = 200; det3.classId = 1; det3.confidence = 0.5;
          detections.push_back(det3);
        }

        return detections;
    }

private:
    bool isValid_ = false;
};

// Structure to store detection information
struct Detection {
    int classId;
    float confidence;
    cv::Rect box;
    cv::Point bottomCenter;
};

// Function to calculate the bottom center point of a bounding box
cv::Point calculateBottomCenter(const cv::Rect& box) {
    return cv::Point(box.x + box.width / 2, box.y + box.height);
}

// Function to find the most centered person (horizontally)
Detection findMostCenteredPerson(const std::vector<Detection>& detections, int frameWidth) {
    if (detections.empty()) {
        return {}; // Return empty Detection if no people found
    }

    Detection mostCenteredPerson;
    int minDistance = INT_MAX;

    for (const auto& det : detections) {
        // Calculate distance from center of the frame (horizontally)
        int distance = std::abs(det.bottomCenter.x - frameWidth / 2);
        if (distance < minDistance) {
            minDistance = distance;
            mostCenteredPerson = det;
        }
    }

    return mostCenteredPerson;
}

// Callback function for eCAL image data
void imageCallback(const char* topic_name, const std::string& msg) {
    static int frameCount = 0;
    frameCount++;
    
    // Convert eCAL message to OpenCV image
    cv::Mat frame = cv::imdecode(cv::Mat(1, msg.size(), CV_8UC1, (void*)msg.data()), cv::IMREAD_COLOR);
    if (frame.empty()) {
        std::cerr << "Error: Could not decode image from eCAL message." << std::endl;
        return;
    }

    // Initialize YOLOv4 detector (only once)
    static Yolov4Detector detector("data/yolov4/yolov4.weights", "data/yolov4/yolov4.cfg", "data/yolov4/coco.names");
    if(!detector.isValid()){
        std::cerr << "Error: Yolov4 model not loaded." << std::endl;
        return;
    }

    // Perform detection
    std::vector<Detection> detections;
    std::vector<Yolov4Detector::Detection> yolov4_detections = detector.detect(frame);
    
    for(const auto& det: yolov4_detections){
      if(det.classId == 0){ //classId 0 is person
        Detection personDetection;
        personDetection.classId = det.classId;
        personDetection.confidence = det.confidence;
        personDetection.box = cv::Rect(det.x, det.y, det.width, det.height);
        personDetection.bottomCenter = calculateBottomCenter(personDetection.box);
        detections.push_back(personDetection);
      }
    }

    // Draw bounding boxes and labels
    for (const auto& det : detections) {
        cv::rectangle(frame, det.box, cv::Scalar(0, 255, 0), 2);
        std::string label = "Person: " + std::to_string(det.confidence);
        cv::putText(frame, label, cv::Point(det.box.x, det.box.y - 10), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
        cv::circle(frame, det.bottomCenter, 5, cv::Scalar(0, 0, 255), -1);
    }

    // Find the most centered person
    Detection mostCenteredPerson = findMostCenteredPerson(detections, frame.cols);

    // Output the bottom center of the most centered person
    if (mostCenteredPerson.confidence > 0) {
        std::cout << "Frame: " << frameCount << " - Most Centered Person Bottom Center: ("
                  << mostCenteredPerson.bottomCenter.x << ", " << mostCenteredPerson.bottomCenter.y << ")"
                  << std::endl;
        // Optionally, draw a distinct marker for the most centered person
        cv::circle(frame, mostCenteredPerson.bottomCenter, 10, cv::Scalar(255, 0, 0), -1);

    } else {
        std::cout << "Frame: " << frameCount << " - No person detected" << std::endl;
    }

    // Show the image
    cv::imshow("Person Detection", frame);
    cv::waitKey(1);
}

int main(int argc, char** argv) {
    // Initialize eCAL
    eCAL::Initialize(argc, argv, "person_detection_node");

    // Check if eCAL is initialized properly
    if (!eCAL::Ok()) {
        std::cerr << "Error: eCAL initialization failed." << std::endl;
        return 1;
    }
    
    // Create an eCAL subscriber for the image topic
    eCAL::string::CSubscriber imageSub("camera_image"); 
    
    // Set the callback function for the subscriber
    imageSub.AddReceiveCallback(imageCallback);

    std::cout << "Person Detection Node started. Waiting for image data..." << std::endl;

    // Keep the node running until eCAL is finalized
    while (eCAL::Ok()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Finalize eCAL
    eCAL::Finalize();

    return 0;
}
