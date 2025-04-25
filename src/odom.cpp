#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>


using namespace cv;
using namespace std;
static ros::Subscriber fisheye_sub;
static cv::Mat img;
static cv_bridge::CvImageConstPtr imageptr;

void image_callback(const sensor_msgs::Image::ConstPtr& imagemsg)
{
    try
    {
        imageptr  = cv_bridge::toCvCopy(imagemsg, imagemsg->encoding);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    img = imageptr->image;

    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    image_u8_t image_april = {
        .width = img_gray.cols,
        .height = img_gray.rows,
        .stride = img_gray.cols,
        .buf = img_gray.data
    };

    // detector here
    apriltag_family_t* tf = tag36h11_create();
    apriltag_detector_t* td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);

    // detect here
    zarray_t* detections = apriltag_detector_detect(td, &image_april);

    std::cout << zarray_size(detections) << std::endl;

    // // Print and draw detections
    // for (int i = 0; i < zarray_size(detections); i++) {
    //     apriltag_detection_t* det;
    //     zarray_get(detections, i, &det);

    //     ROS_INFO("Detected tag ID: %d", det->id);

    //     // Draw outline
    //     for (int j = 0; j < 4; j++) {
    //         cv::line(img,
    //             cv::Point(det->p[j][0], det->p[j][1]),
    //             cv::Point(det->p[(j+1)%4][0], det->p[(j+1)%4][1]),
    //             cv::Scalar(0, 255, 0), 2);
    //     }

    //     // Draw tag center
    //     cv::circle(img, cv::Point(det->c[0], det->c[1]), 5, cv::Scalar(0, 0, 255), -1);

    //     // Put tag ID
    //     cv::putText(img, to_string(det->id), cv::Point(det->c[0], det->c[1]),
    //                 cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 0), 2);
    // }

    // // Show image
    // cv::imshow("AprilTag Detection", img);
    // cv::waitKey(10);

    // Cleanup
    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "odom");
    ros::NodeHandle nh;

    fisheye_sub = nh.subscribe<sensor_msgs::Image>
        ("/stereo_camera/raw", 1, &image_callback);
    
        
    // image_u8_t* im = image_u8_create_from_pnm("test.pnm");
    // if (im == NULL) {
    //     fprintf(stderr, "Failed to load pnm image.\n");
    //     exit(1);
    // }
    // apriltag_detector_t *td = apriltag_detector_create();
    // apriltag_family_t *tf = tag36h11_create();
    // apriltag_detector_add_family(td, tf);
    // zarray_t *detections = apriltag_detector_detect(td, im);
    
    // for (int i = 0; i < zarray_size(detections); i++) {
    //     apriltag_detection_t *det;
    //     zarray_get(detections, i, &det);
    
    //     // Do stuff with detections here.
    // }
    // // Cleanup.
    // apriltag_detections_destroy(detections);
    // tag36h11_destroy(tf);
    // apriltag_detector_destroy(td);

    ros::spin();
    
    return 0;
    
    
}