#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>

#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>


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
    
    cv::imshow("test", img);
    cv::waitKey(10);
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
    // apriltag_family_t *tf = tagStandard41h12_create();
    // apriltag_detector_add_family(td, tf);
    // zarray_t *detections = apriltag_detector_detect(td, im);
    
    // for (int i = 0; i < zarray_size(detections); i++) {
    //     apriltag_detection_t *det;
    //     zarray_get(detections, i, &det);
    
    //     // Do stuff with detections here.
    // }
    // // Cleanup.
    // apriltag_detections_destroy(detections);
    // tagStandard41h12_destroy(tf);
    // apriltag_detector_destroy(td);

    ros::spin();
    
    return 0;
}