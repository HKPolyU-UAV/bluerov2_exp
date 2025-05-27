// control with apriltag frame

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>
#include <geometry_msgs/TwistStamped.h>

static mavros_msgs::State current_state;
static geometry_msgs::Twist twist_lala;
static geometry_msgs::Pose pose_lala;

static Eigen::Vector3d p_gain, d_gain = Eigen::Vector3d::Zero();
static Eigen::Vector3d posi_now = Eigen::Vector3d::Zero();
static Eigen::Vector4d q_now = Eigen::Vector4d::Zero();

static Eigen::Vector3d error_now, error_prev = Eigen::Vector3d::Zero();

static int counter = 0;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    posi_now.x() = msg->pose.position.x;
    posi_now.y() = msg->pose.position.y;
    posi_now.z() = msg->pose.position.z;

    q_now.w() = msg->pose.orientation.w;
    q_now.x() = msg->pose.orientation.x;
    q_now.y() = msg->pose.orientation.y;
    q_now.z() = msg->pose.orientation.z;
}

Eigen::Vector3d get_cmd(const Eigen::Vector3d setpt)
{
    counter ++;
    error_now = setpt - posi_now;

    if (counter < 2)
        error_prev = error_now;

    Eigen::Vector3d sig = p_gain.cwiseProduct(error_now) + d_gain.cwiseProduct(error_now - error_prev) / 0.05;
    error_prev = error_now;

    return sig;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1, true);

    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/mocap/pose", 10, pose_cb);

    // ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
    //         ("mavros/cmd/arming");
    // ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            // ("mavros/set_mode");

    ros::Rate rate(20.0);

    pose_lala.position.x = 4.3;
    pose_lala.position.y = 2.0;
    pose_lala.position.z = 0.47;

    while(ros::ok())
    {
        pose_lala.orientation.w = 0.0;
        pose_lala.orientation.x = 0.0;
        pose_lala.orientation.y = 0.0;
        pose_lala.orientation.z = 1;

        local_pos_pub.publish(pose_lala);

        if (current_state.armed)
        {
            std::cout << "SETVEL HERE" << std::endl;
            std::cout << "x: " << twist_lala.angular.x << std::endl;
            std::cout << "y: " << twist_lala.angular.y << std::endl;
            std::cout << "z: " << twist_lala.angular.z << std::endl;
        }

        ros::spinOnce();
        rate.sleep();
     }
 
     return 0;
 }
