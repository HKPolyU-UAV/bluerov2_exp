#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{   
    // Node Initialization
    ros::init(argc, argv, "publish_camera");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Parameters
    int img_w{1920};
    int img_h{960};
    int fps{30};
    int queue_size{10};
    bool auto_exposure{true};
    int exposure_time{-6};
    bool auto_white_balance{true};
    string pubstr{"/stereo_camera/raw"};
    string image_format{"mjpg"};
    cv_bridge::CvImage img_bridge;
    int img_counter{0};
    cv::Mat cv_frame;
    sensor_msgs::Image img_msg;
    string cam_port_name{"/dev/video0"};

    // Load parameters from ROS parameter server
    nh.getParam("img_width", img_w);
    nh.getParam("img_height", img_h);
    nh.getParam("frame_rate", fps);
    nh.getParam("auto_exposure", auto_exposure);
    nh.getParam("exposure_time", exposure_time);
    nh.getParam("auto_white_balance", auto_white_balance);
    
    if (nh.hasParam("topic_name"))
        nh.getParam("topic_name", pubstr);

    if (nh.hasParam("image_format"))
        nh.getParam("image_format", image_format);

    if (nh.hasParam("cam_port_name"))
        nh.getParam("cam_port_name", cam_port_name);

    if (nh.hasParam("queue_size"))
        nh.getParam("queue_size", queue_size);

    // Initialize Camera
    cv::VideoCapture cap(cam_port_name, cv::CAP_V4L2);
    
    if (!cap.isOpened()) {
        ROS_ERROR("Camera is not opened");
        return -1;
    } else {
        ROS_INFO("Camera opened successfully");
    }

    // Set Image Format
    if (image_format == "mjpg") {
        std::cout << "Image Format: MJPG: " 
                  << cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')) 
                  << std::endl;
    } else {
        std::cout << "Image Format: YUYV: " 
                  << cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V')) 
                  << std::endl;
    }

    // Set Camera Properties
    cap.set(cv::CAP_PROP_FRAME_WIDTH, img_w);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, img_h);
    cap.set(cv::CAP_PROP_FPS, fps);

    std::cout << "FPS: " << cap.get(cv::CAP_PROP_FPS) << std::endl;
    std::cout << "Image Width: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
    std::cout << "Image Height: " << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;

    // Set Exposure
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);  // Enable Auto Exposure
    if (!auto_exposure) {
        std::cout << "EXPOSURE RESET" << std::endl;
        cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);  // Manual Exposure
        cap.set(cv::CAP_PROP_EXPOSURE, exposure_time);
    }

    // Set White Balance
    cap.set(cv::CAP_PROP_AUTO_WB, 3);  // Enable Auto White Balance
    if (!auto_white_balance) {
        std::cout << "WHITE BALANCE RESET" << std::endl;
        cap.set(cv::CAP_PROP_AUTO_WB, 1);
        cap.set(cv::CAP_PROP_WB_TEMPERATURE, 6500);
    }

    // Print Exposure and White Balance Settings
    std::cout << "Auto Exposure: " << cap.get(cv::CAP_PROP_AUTO_EXPOSURE) << std::endl;
    std::cout << "Exposure Time: " << cap.get(cv::CAP_PROP_EXPOSURE) << std::endl;
    std::cout << "Auto White Balance: " << cap.get(cv::CAP_PROP_AUTO_WB) << std::endl;
    std::cout << "White Balance Temperature: " << cap.get(cv::CAP_PROP_WB_TEMPERATURE) << std::endl;

    // Publisher Initialization
    image_transport::Publisher rawpub = it.advertise(pubstr, queue_size);

    // Start Image Capture Loop
    ros::Rate loop_rate(fps);
    while (ros::ok()) {
        if (!cap.read(cv_frame)) {
            ROS_WARN("No image detected");
            continue;
        }

        // Convert to RGB (from BGR)
        // cv::cvtColor(cv_frame, cv_frame, cv::COLOR_BGR2RGB);

        // Create ROS Image Message
        std_msgs::Header header;
        header.seq = img_counter++;
        header.stamp = ros::Time::now();
        
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, cv_frame);
        img_bridge.toImageMsg(img_msg);
        rawpub.publish(img_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}