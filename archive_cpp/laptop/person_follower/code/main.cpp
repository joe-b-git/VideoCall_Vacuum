#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <ecal/msg/protobuf/publisher.h>

#include "image.pb.h"
#include "movement.pb.h"

class YoloDetector {
public:
    YoloDetector(const std::string& data_dir) {
        std::string yolo_weights_path;
        std::string yolo_config_path;

        if (!std::filesystem::exists(data_dir)) {
            throw std::runtime_error("Data directory not found: " + data_dir);
        }

        if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
            yolo_weights_path = data_dir + "/yolov4.weights";
            yolo_config_path = data_dir + "/yolov4.cfg";
        } else {
            yolo_weights_path = data_dir + "/yolov4-tiny.weights";
            yolo_config_path = data_dir + "/yolov4-tiny.cfg";
        }

        if (!std::filesystem::exists(yolo_weights_path)) {
            throw std::runtime_error("YOLO weights file not found: " + yolo_weights_path);
        }
        if (!std::filesystem::exists(yolo_config_path)) {
            throw std::runtime_error("YOLO config file not found: " + yolo_config_path);
        }

        try {
            net = cv::dnn::readNet(yolo_weights_path, yolo_config_path);
        } catch (const cv::Exception& e) {
            throw std::runtime_error("Error loading YOLO model: " + std::string(e.what()));
        }

        if (cv::cuda::getCudaEnabledDeviceCount() > 0) {
            net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
            net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
        }

        layer_names = net.getLayerNames();
        std::vector<int> output_layers_indices = net.getUnconnectedOutLayers();
        output_layers.resize(output_layers_indices.size());
        for (size_t i = 0; i < output_layers_indices.size(); ++i) {
            output_layers[i] = layer_names[output_layers_indices[i] - 1];
        }

        person_class_id = 0;
        confidence_threshold = 0.5;
        nms_threshold = 0.4;
        prev_detections.clear();
        frame_count = 0;
    }

    std::tuple<std::vector<cv::Rect>, std::vector<cv::Point2f>, int> detect_people(cv::Mat& img) {
        int height = img.rows;
        int width = img.cols;

        if (frame_count % 2 == 0 || true) {
            cv::Mat blob = cv::dnn::blobFromImage(img, 0.00392, cv::Size(416, 416), cv::Scalar(0, 0, 0), true, false);
            net.setInput(blob);
            net.forward(prev_detections, output_layers);
        }

        frame_count++;

        std::vector<int> class_ids;
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        std::vector<cv::Mat> outs = prev_detections;
        for (const auto& out : outs) {
          for (int i = 0; i < out.rows; ++i)
          {
            cv::Mat detection = out.row(i);
            cv::Mat scores = detection.colRange(5, detection.cols);
            cv::Point class_id_point;
            double confidence;
            cv::minMaxLoc(scores, 0, &confidence, 0, &class_id_point);

            if (confidence > confidence_threshold)
            {
                int center_x = (int)(detection.at<float>(0) * width);
                int center_y = (int)(detection.at<float>(1) * height);
                int w = (int)(detection.at<float>(2) * width);
                int h = (int)(detection.at<float>(3) * height);
                int x = center_x - w / 2;
                int y = center_y - h / 2;
                boxes.push_back(cv::Rect(x, y, w, h));
                confidences.push_back((float)confidence);
                class_ids.push_back(class_id_point.x);
            }
          }
        }


        std::vector<int> indexes;
        cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold, nms_threshold, indexes);

        std::vector<cv::Rect> people_boxes;
        std::vector<cv::Point2f> people_positions;
        for (size_t i = 0; i < indexes.size(); ++i) {
            int idx = indexes[i];
            if (class_ids[idx] == person_class_id) {
                cv::Rect box = boxes[idx];
                people_boxes.push_back(box);
                float top_center_x = box.x + box.width / 2.0f;
                float top_center_y = box.y;
                people_positions.push_back(cv::Point2f(top_center_x, top_center_y));
            }
        }

        return std::make_tuple(people_boxes, people_positions, width);
    }

private:
    cv::dnn::Net net;
    std::vector<std::string> layer_names;
    std::vector<std::string> output_layers;
    int person_class_id;
    float confidence_threshold;
    float nms_threshold;
    std::vector<cv::Mat> prev_detections;
    int frame_count;
};

class PersonFollower {
public:
  PersonFollower(const std::string& data_dir): detector(data_dir), movement_publisher("movement_command") {
    prev_yaw_rate = 0.0;
  }
  //New onImage function, now uses a ImageMessage
    void on_image(const ImageMessage& image_msg){
        // Convert received data to cv::Mat
        cv::Mat frame(
            image_msg.height(),
            image_msg.width(),
            CV_8UC3,  // Assuming 3 channels (BGR) format
            const_cast<unsigned char*>(reinterpret_cast<const unsigned char*>(image_msg.data().c_str()))
        );
       
      std::vector<cv::Rect> people_boxes;
      std::vector<cv::Point2f> people_positions;
      int width;
      std::tie(people_boxes, people_positions, width) = detector.detect_people(frame);
      float center_width = width / 2.0f;

      cv::Point2f most_centered_person_position;
      float closest_person_distance = std::numeric_limits<float>::infinity();
      for (size_t i = 0; i < people_boxes.size(); ++i) {
          cv::Rect box = people_boxes[i];
          cv::Point2f pos = people_positions[i];
          cv::rectangle(frame, box, cv::Scalar(0, 255, 255), 2);
          float distance_to_center = std::abs(center_width - pos.x);
          if (distance_to_center < closest_person_distance) {
              closest_person_distance = distance_to_center;
              most_centered_person_position = pos;
          }
      }

      float yaw_rate = 0.0f;
      float velocity = 0.0f;

      if (!people_boxes.empty()) {
          float top_center_x = most_centered_person_position.x;

          if (top_center_x < center_width - 180) {
              yaw_rate = 30.0f;
          } else if (top_center_x < center_width - 120) {
              yaw_rate = 15.0f;
          } else if (top_center_x < center_width - 80) {
              yaw_rate = 7.5f;
          } else if (top_center_x > center_width + 80) {
              yaw_rate = -7.5f;
          } else if (top_center_x > center_width + 120) {
              yaw_rate = -15.0f;
          } else if (top_center_x > center_width + 180) {
              yaw_rate = -30.0f;
          }

          float top_center_y = most_centered_person_position.y;

          if (top_center_y < 20) {
              velocity = -0.1f;
          } else if (top_center_y > 80) {
              velocity = 0.1f;
          } else {
              velocity = 0.0f;
          }
      }

      if (yaw_rate != prev_yaw_rate) {
          Movement movement_message;
          movement_message.set_velocity(velocity);
          movement_message.set_yaw_rate(yaw_rate);
          movement_publisher.Send(movement_message);
          prev_yaw_rate = yaw_rate;
      }
      cv::imshow("Webcam Feed", frame);
      cv::waitKey(1);
    }
    //Callback
     void on_image_callback(const std::string& topic_name, const eCAL::protobuf::Sample<ImageMessage>&& msg) {
        on_image(msg.get());
    }

  private:
  YoloDetector detector;
  eCAL::protobuf::CPublisher<Movement> movement_publisher;
  float prev_yaw_rate;
};

int main(int argc, char** argv) {
    eCAL::Initialize(argc, argv, "PersonFollower");
    eCAL::Util::EnableLoopback(true);
    
    std::string data_dir = "./data";
    try{
        
      PersonFollower personFollower(data_dir);
      eCAL::protobuf::CSubscriber<ImageMessage> subscriber("webcam_feed");

      // Set callback, now uses on_image_callback
      subscriber.AddReceiveCallback(std::bind(&PersonFollower::on_image_callback, &personFollower, std::placeholders::_1, std::placeholders::_2));

      cv::namedWindow("Webcam Feed", cv::WINDOW_AUTOSIZE);
      while (eCAL::Ok()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    catch(const std::runtime_error &e){
        std::cerr << e.what() << std::endl;
        return 1;
    }
    cv::destroyAllWindows();
    eCAL::Finalize();

    return 0;
}
