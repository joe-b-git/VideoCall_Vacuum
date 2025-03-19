#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <vector>

// Assuming you have a proto message for the image data
#include "image.pb.h"

int main() {
    // Initialize eCAL
    eCAL::Initialize(0, nullptr, "WebcamPublisher");

    // Create publisher
    eCAL::protobuf::CPublisher<ImageMessage> publisher("webcam_feed");

    // Open the default camera (usually USB webcam is 0)
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
        return -1;
    }

    // --- Optimization 1: Reduce Resolution ---
    // Reduce resolution to lighten processing and data transfer load.
    // Experiment with values lower than 640x480 if possible.
    int width = 640; // Reduced width
    int height = 480; // Reduced height
    cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);

    // --- Optimization 2: Use MJPEG if available ---
    // MJPEG compression reduces the bandwidth required.
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); 
    
    // --- Optimization 3: Set FPS Directly ---
    // Try to set the desired FPS on the camera directly.
    // This may or may not be supported by your camera.
    // it will fail silently, if it is not supported.
    cap.set(cv::CAP_PROP_FPS, 30.0); // Target 30 FPS

    ImageMessage img_msg;
    cv::Mat frame;

    // --- Optimization 4: Pre-allocate Data Buffer ---
    // Avoid repeated memory allocations inside the loop.
    int frame_size = width * height * 3; // Assuming 3 channels (BGR)
    std::vector<uchar> buffer(frame_size);
    img_msg.mutable_data()->reserve(frame_size);

    // --- Optimization 5: reduced sleep time---
    // we removed sleep_for in order to get as many frames as possible

    while (eCAL::Ok()) {
        // Capture frame
        if (!cap.read(frame)) {
            std::cerr << "Error: Could not read frame." << std::endl;
            break;
        }

        // --- Optimization 6: Minimize Data Copying ---
        // Use `memcpy` for faster copying.
        //Ensure buffer size
        if(frame.isContinuous()){
            frame_size = frame.total() * frame.elemSize();
        } else{
            frame_size = width * height * 3;
        }
        
        if (frame_size > buffer.size()){
            buffer.resize(frame_size);
        }
        memcpy(buffer.data(), frame.data, frame_size);

        // Convert frame to message (now using pre-allocated buffer)
        img_msg.set_timestamp(eCAL::Time::GetMicroSeconds());
        img_msg.set_width(frame.cols);
        img_msg.set_height(frame.rows);
        img_msg.set_channels(frame.channels());
        img_msg.set_data(buffer.data(), frame_size);

        // Publish the message
        publisher.Send(img_msg);
    }

    // Cleanup
    cap.release();
    eCAL::Finalize();

    return 0;
}
