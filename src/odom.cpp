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
static ros::Subscriber watereye_sub;
static cv::Mat img;
static cv_bridge::CvImageConstPtr imageptr;
static cv::Mat camMat = cv::Mat::eye(3, 3, CV_64F);
static std::vector<cv::Point3d> pts_3d_final;
static std::vector<cv::Point2d> pts_2d_final;
static cv::Vec3d rvec, tvec;
static cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

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

    pts_2d_final.clear();

    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        for (int j = 0; j < 4; j++)
        {
            cv::circle(img, cv::Point(det->p[j][0], det->p[j][1]), 5, cv::Scalar(0, 255, 0), -1);
            // std::cout<<cv::Point(det->p[j][0], det->p[j][1])<<std::endl<<std::endl;
            pts_2d_final.emplace_back(cv::Point2d(det->p[j][0], det->p[j][1]));
        }
        cv::circle(img, cv::Point(det->c[0], det->c[1]), 5, cv::Scalar(0, 0, 255), -1);
    }

    cv::solvePnP(pts_3d_final, pts_2d_final, camMat, distCoeffs, rvec, tvec, cv::SOLVEPNP_ITERATIVE);
    
    // cv::imshow("AprilTag Detection", img);
    // cv::waitKey(5);

    cv::Mat rmat = cv::Mat::eye(3,3,CV_64F);
    cv::Rodrigues(rvec, rmat);

    // std::cout<<"POSE HERE"<<std::endl;
    // std::cout<<rmat<<std::endl;
    // std::cout<<tvec<<std::endl;
    // std::cout<<std::endl<<std::endl;
    
    apriltag_detections_destroy(detections);
    apriltag_detector_destroy(td);
    tag36h11_destroy(tf);
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "odom");
    ros::NodeHandle nh;

    watereye_sub = nh.subscribe<sensor_msgs::Image>
        ("/stereo_camera/raw", 1, &image_callback);

    XmlRpc::XmlRpcValue intrinsics_list;

    std::cout<<1<<std::endl;
    
    nh.getParam("cam_intrinsics", intrinsics_list);
    std::cout<<1<<std::endl;

    for (int i = 0; i < 4; i ++)
    {
        std::cout<<intrinsics_list[i]<<std::endl;
    }

    std::cout<<"loop"<<std::endl;

    camMat.at<float>(0, 0) = static_cast<float>(static_cast<double>(intrinsics_list[0]));
    camMat.at<float>(0, 1) = static_cast<float>(static_cast<double>(intrinsics_list[1]));
    camMat.at<float>(1, 1) = static_cast<float>(static_cast<double>(intrinsics_list[2]));
    camMat.at<float>(1, 2) = static_cast<float>(static_cast<double>(intrinsics_list[3]));


    double tag_size = 0.3;

    pts_3d_final.emplace_back(cv::Point3d(-tag_size / 2,  0.0, -tag_size / 2));
    pts_3d_final.emplace_back(cv::Point3d(tag_size / 2,  0.0, -tag_size / 2));
    pts_3d_final.emplace_back(cv::Point3d(tag_size / 2,  0.0, tag_size / 2));
    pts_3d_final.emplace_back(cv::Point3d(-tag_size / 2, 0.0, tag_size / 2));

    std::cout<<1<<std::endl;

    std::cout<<"INITIALIZATION DONE"<<std::endl;
    
    ros::spin();
    
    return 0;
    

}