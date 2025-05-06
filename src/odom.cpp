#include <iostream>
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>

#include <image_transport/image_transport.h>

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>

static ros::Subscriber watereye_sub;
static image_transport::Publisher image_pub;
static ros::Publisher pose_pub;

static cv::Mat img;
static cv_bridge::CvImageConstPtr imageptr;
static cv::Mat camMat = cv::Mat::eye(3, 3, CV_64F);
static cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

static std::vector<cv::Point3d> pts_3d_final;
static std::vector<cv::Point2d> pts_2d_final;

static cv::Vec3d rvec, tvec; 
static cv::Mat rmat = cv::Mat::eye(3,3,CV_64F);

static geometry_msgs::PoseStamped pose_obj;
static Eigen::Matrix3d rot_SO3 = Eigen::Matrix3d::Identity();
static Eigen::Matrix3d cam_to_body_rot = Eigen::Matrix3d::Identity();
static Eigen::Vector3d posi = Eigen::Vector3d::Zero();
static Eigen::Quaterniond q;

void apriltag_detect(image_u8_t& image_april);
void get_pose();

static apriltag_family_t* tf = NULL;
static apriltag_detector_t* td = NULL;
static pthread_mutex_t detector_mutex = PTHREAD_MUTEX_INITIALIZER;

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

    // double tick = ros::Time::now().toSec();

    std::cout<<"==========="<<std::endl;

    apriltag_detect(image_april);

    // double tock = ros::Time::now().toSec();
    // std::cout<<1/(tock - tick)<<std::endl<<std::endl;;

    // tick = ros::Time::now().toSec();
    get_pose();
    // tock = ros::Time::now().toSec();
    // std::cout<<1/(tock - tick)<<std::endl<<std::endl;;

    pose_pub.publish(pose_obj);
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "odom");
    ros::NodeHandle nh;

    watereye_sub = nh.subscribe<sensor_msgs::Image>
        ("/stereo_camera/raw", 1, &image_callback);

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/mocap/pose", 1);

    XmlRpc::XmlRpcValue intrinsics_list;
    Eigen::Vector4d intrinsics_value;
    nh.getParam("cam_intrinsics", intrinsics_list);

    for(int i = 0; i < 4; i++)
        intrinsics_value[i] = intrinsics_list[i];

    camMat.at<double>(0, 0) = intrinsics_value[0];
    camMat.at<double>(0, 2) = intrinsics_value[1];
    camMat.at<double>(1, 1) = intrinsics_value[2];
    camMat.at<double>(1, 2) = intrinsics_value[3];

    double tag_size = 0.3;
    nh.getParam("tag_size", tag_size);

    pts_3d_final.emplace_back(cv::Point3d(0.0, -tag_size / 2, -tag_size / 2));
    pts_3d_final.emplace_back(cv::Point3d(0.0,  tag_size / 2, -tag_size / 2));
    pts_3d_final.emplace_back(cv::Point3d(0.0,  tag_size / 2, tag_size / 2));
    pts_3d_final.emplace_back(cv::Point3d(0.0, -tag_size / 2, tag_size / 2));

    std::cout<<"ODOM INITIALIZATION DONE"<<std::endl;
    std::cout<<"INTRISICS HERE"<<std::endl;
    std::cout << camMat << std::endl;

    cam_to_body_rot << 
        0, 0, 1,
        -1, 0, 0,
        0, -1, 0;

    pthread_mutex_lock(&detector_mutex);

    if (td == NULL) {
        tf = tag36h11_create();
        td = apriltag_detector_create();
        apriltag_detector_add_family(td, tf);
    }
    pthread_mutex_unlock(&detector_mutex);

    ros::spin();
    
    return 0;
}

void apriltag_detect(image_u8_t& image_april)
{
    double tick = ros::Time::now().toSec();
    
    // detect here
    pthread_mutex_lock(&detector_mutex);
    zarray_t* detections = apriltag_detector_detect(td, &image_april);
    pthread_mutex_unlock(&detector_mutex);

    std::cout<<"hi"<<std::endl;
    std::cout<<1.0/(ros::Time::now().toSec() - tick)<<std::endl;

    pts_2d_final.clear();
    for (int i = 0; i < zarray_size(detections); i++)
    {
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);
        for (int j = 0; j < 4; j++)
        {
            // cv::circle(img, cv::Point(det->p[j][0], det->p[j][1]), 5, cv::Scalar(0, 255, 0), -1);
            pts_2d_final.emplace_back(cv::Point2d(det->p[j][0], det->p[j][1]));
        }
        // cv::circle(img, cv::Point(det->c[0], det->c[1]), 5, cv::Scalar(0, 0, 255), -1);
    }

    
}

void get_pose()
{
    if (pts_2d_final.size() != pts_3d_final.size())
        return;
    if (pts_2d_final.size() == 0 || pts_3d_final.size() == 0)
        return;

    cv::solvePnP(pts_3d_final, pts_2d_final, camMat, distCoeffs, rvec, tvec, cv::SOLVEPNP_ITERATIVE);

    cv::Rodrigues(rvec, rmat);

    rot_SO3 <<
        rmat.at<double>(0,0), rmat.at<double>(0,1), rmat.at<double>(0,2),
        rmat.at<double>(1,0), rmat.at<double>(1,1), rmat.at<double>(1,2),
        rmat.at<double>(2,0), rmat.at<double>(2,1), rmat.at<double>(2,2);
    // T_SE3_I2B;

    posi = cam_to_body_rot * Eigen::Vector3d(tvec(0), tvec(1), tvec(2));
    rot_SO3 = cam_to_body_rot * rot_SO3.eval();
    q = Eigen::Quaterniond(rot_SO3);

    pose_obj.header.stamp = ros::Time::now();
    pose_obj.header.frame_id = "map";
    pose_obj.pose.position.x = posi.x();
    pose_obj.pose.position.y = posi.y();
    pose_obj.pose.position.z = posi.z();

    pose_obj.pose.orientation.w = q.w();
    pose_obj.pose.orientation.x = q.x();
    pose_obj.pose.orientation.y = q.y();
    pose_obj.pose.orientation.z = q.z();
}
