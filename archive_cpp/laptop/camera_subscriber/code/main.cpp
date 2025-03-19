#include <ecal/ecal.h>
#include <ecal/msg/protobuf/subscriber.h>
#include <opencv2/opencv.hpp>
#include <iostream>

// Include the same proto message definition
#include "image.pb.h"

void OnImage(const ImageMessage& msg)
{
    // Convert received data to cv::Mat
    cv::Mat frame(
        msg.height(),
        msg.width(),
        CV_8UC3,  // Assuming 3 channels (BGR) format
        (void*)msg.data().data()
    );
    
    // Show the frame
    cv::imshow("Webcam Feed", frame);
    cv::waitKey(1);  // Process GUI events
}

int main()
{
    // Initialize eCAL
    eCAL::Initialize(0, nullptr, "WebcamSubscriber");
    
    // Create subscriber
    eCAL::protobuf::CSubscriber<ImageMessage> subscriber("webcam_feed");
    
    // Set callback
    subscriber.AddReceiveCallback(std::bind(&OnImage, std::placeholders::_2));
    
    // Create window
    cv::namedWindow("Webcam Feed", cv::WINDOW_AUTOSIZE);
    
    // Main loop
    while (eCAL::Ok())
    {
        // Sleep to prevent busy waiting
        eCAL::Process::SleepMS(100);
    }
    
    // Cleanup
    cv::destroyAllWindows();
    eCAL::Finalize();
    
    return 0;
}
