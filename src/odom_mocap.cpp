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

static ros::Subscriber watereye_sub, real_sub;
static image_transport::Publisher image_pub;
static ros::Publisher pose_pub, originpose_pub, bluepose_pub;

static cv::Mat img;
static cv_bridge::CvImageConstPtr imageptr;
static cv::Mat camMat = cv::Mat::eye(3, 3, CV_64F);
static cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

static std::vector<cv::Point3d> pts_3d_final_origin, pts_3d_final_blue;
static std::vector<cv::Point2d> pts_2d_final_origin, pts_2d_final_blue;

static cv::Vec3d rvec, tvec; 
static cv::Mat rmat = cv::Mat::eye(3,3,CV_64F);

static geometry_msgs::PoseStamped pose_obj;
static Eigen::Matrix3d rot_SO3 = Eigen::Matrix3d::Identity();
static Eigen::Matrix3d cam_to_body_rot = Eigen::Matrix3d::Identity();
static Eigen::Vector3d posi = Eigen::Vector3d::Zero();
static Eigen::Quaterniond q;

void apriltag_detect(image_u8_t& image_april);
void get_campose();
void get_bluepose();

static apriltag_family_t* tf = NULL;
static apriltag_detector_t* td = NULL;
static pthread_mutex_t detector_mutex = PTHREAD_MUTEX_INITIALIZER;


static Eigen::Matrix4d origin_SE3, blue_SE3, blue_in_origin_SE3;

void get_delta()
{

    blue_in_origin_SE3 = origin_SE3.inverse() * blue_SE3;

    posi = blue_in_origin_SE3.block<3,1>(0,3);
    q = Eigen::Quaterniond(blue_in_origin_SE3.block<3,3>(0,0));

    pose_obj.header.stamp = ros::Time::now();
    pose_obj.header.frame_id = "map";
    pose_obj.pose.position.x = posi.x();
    pose_obj.pose.position.y = posi.y();
    pose_obj.pose.position.z = posi.z();

    pose_obj.pose.orientation.w = q.w();
    pose_obj.pose.orientation.x = q.x();
    pose_obj.pose.orientation.y = q.y();
    pose_obj.pose.orientation.z = q.z();

    pose_pub.publish(pose_obj);
}

void image_callback(const sensor_msgs::Image::ConstPtr& imagemsg)
{
    return;
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

    apriltag_detect(image_april);

    get_campose();
    get_bluepose();
    get_delta();
}


void image_compressed_callback(const sensor_msgs::CompressedImage::ConstPtr &msg)
{
    try
    {
        img = cv::imdecode(cv::Mat(msg->data), 1);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    cv::Mat img_gray;
    cv::cvtColor(img, img_gray, cv::COLOR_BGR2GRAY);

    image_u8_t image_april = {
        .width = img_gray.cols,
        .height = img_gray.rows,
        .stride = img_gray.cols,
        .buf = img_gray.data
    };

    apriltag_detect(image_april);

    get_campose();
    get_bluepose();
    get_delta();
}

int main(int argc, char** argv)
{   
    ros::init(argc, argv, "odom_mocap");
    ros::NodeHandle nh;

    watereye_sub = nh.subscribe<sensor_msgs::Image>
        ("/stereo_camera/raw", 1, &image_callback);

    real_sub = nh.subscribe<sensor_msgs::CompressedImage>
        ("/camera/color/image_raw/compressed", 1, &image_compressed_callback);

    bluepose_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/blue_pose", 1);

    pose_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/mavros/mocap/pose", 1);

    originpose_pub = nh.advertise<geometry_msgs::PoseStamped>
        ("/origin_pose", 1);

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

    pts_3d_final_origin.emplace_back(cv::Point3d(-tag_size / 2, -tag_size / 2, 0));
    pts_3d_final_origin.emplace_back(cv::Point3d( tag_size / 2, -tag_size / 2, 0));
    pts_3d_final_origin.emplace_back(cv::Point3d( tag_size / 2,  tag_size / 2, 0));
    pts_3d_final_origin.emplace_back(cv::Point3d(-tag_size / 2,  tag_size / 2, 0));

    pts_3d_final_blue.emplace_back(cv::Point3d(-tag_size / 2,  tag_size / 2, 0));
    pts_3d_final_blue.emplace_back(cv::Point3d(-tag_size / 2, -tag_size / 2, 0));
    pts_3d_final_blue.emplace_back(cv::Point3d( tag_size / 2, -tag_size / 2, 0));
    pts_3d_final_blue.emplace_back(cv::Point3d( tag_size / 2,  tag_size / 2, 0));

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

    pts_2d_final_origin.clear();
    pts_2d_final_blue.clear();
    
    
    for (int i = 0; i < zarray_size(detections); i++)
    {
        
        apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        if (det->id == 1)
            for (int j = 0; j < 4; j++)
            {
                cv::circle(img, cv::Point(det->p[j][0], det->p[j][1]), 5, cv::Scalar(0, 255, 0), -1);
                pts_2d_final_origin.emplace_back(cv::Point2d(det->p[j][0], det->p[j][1]));
            }

        if (det->id == 0)
            for (int j = 0; j < 4; j++)
            {
                cv::circle(img, cv::Point(det->p[j][0], det->p[j][1]), 5, cv::Scalar(255, 255, 0), -1);
                pts_2d_final_blue.emplace_back(cv::Point2d(det->p[j][0], det->p[j][1]));
            }
        
    }    

    cv::imshow("lala", img);
    cv::waitKey(20);
}

void get_campose()
{
    if (pts_2d_final_origin.size() != pts_3d_final_origin.size())
        return;
    if (pts_2d_final_origin.size() == 0 || pts_3d_final_origin.size() == 0)
        return;

    cv::solvePnP(pts_3d_final_origin, pts_2d_final_origin, camMat, distCoeffs, rvec, tvec, cv::SOLVEPNP_ITERATIVE);

    cv::Rodrigues(rvec, rmat);

    rot_SO3 <<
        rmat.at<double>(0,0), rmat.at<double>(0,1), rmat.at<double>(0,2),
        rmat.at<double>(1,0), rmat.at<double>(1,1), rmat.at<double>(1,2),
        rmat.at<double>(2,0), rmat.at<double>(2,1), rmat.at<double>(2,2);
    // T_SE3_I2B;

    posi = cam_to_body_rot * Eigen::Vector3d(tvec(0), tvec(1), tvec(2));
    
    rot_SO3 = cam_to_body_rot * rot_SO3.eval();

    origin_SE3.setIdentity();
    origin_SE3.block<3,3>(0,0) = rot_SO3;
    origin_SE3.block<3,1>(0,3) = posi;

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

    originpose_pub.publish(pose_obj);
}


void get_bluepose()
{
    if (pts_2d_final_blue.size() != pts_3d_final_blue.size())
        return;
    if (pts_2d_final_blue.size() == 0 || pts_3d_final_blue.size() == 0)
        return;

    cv::solvePnP(pts_3d_final_blue, pts_2d_final_blue, camMat, distCoeffs, rvec, tvec, cv::SOLVEPNP_ITERATIVE);

    cv::Rodrigues(rvec, rmat);

    rot_SO3 <<
        rmat.at<double>(0,0), rmat.at<double>(0,1), rmat.at<double>(0,2),
        rmat.at<double>(1,0), rmat.at<double>(1,1), rmat.at<double>(1,2),
        rmat.at<double>(2,0), rmat.at<double>(2,1), rmat.at<double>(2,2);
    // T_SE3_I2B;


    posi = cam_to_body_rot * Eigen::Vector3d(tvec(0), tvec(1), tvec(2));
    
    rot_SO3 = cam_to_body_rot * rot_SO3.eval();

    blue_SE3.setIdentity();
    blue_SE3.block<3,3>(0,0) = rot_SO3;
    blue_SE3.block<3,1>(0,3) = posi;

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

    bluepose_pub.publish(pose_obj);
}

