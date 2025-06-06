#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <Eigen/Dense>
#include <geometry_msgs/TwistStamped.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
current_state = *msg;
}

geometry_msgs::PoseStamped pose_now;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    pose_now = *msg;
} 

int main(int argc, char **argv)
{
ros::init(argc, argv, "offb_node");
ros::NodeHandle nh;

ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
        ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    ros::Subscriber sub_pose = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 1, pose_cb);

    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 1, true);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 1.0;
    pose.pose.position.y = 1.0;
    pose.pose.position.z = pose_now.pose.position.z;

    pose.pose.orientation.w = pose_now.pose.orientation.w;
    pose.pose.orientation.x = pose_now.pose.orientation.x;
    pose.pose.orientation.y = pose_now.pose.orientation.y;
    pose.pose.orientation.z = pose_now.pose.orientation.z;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "GUIDED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    std::cout<<"START BLUEROV2"<<std::endl;

    while(ros::ok()){
        if( current_state.mode != "GUIDED" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        pose.pose.position.x = 1.0;
        pose.pose.position.y = 1.0;
        pose.pose.position.z = pose_now.pose.position.z;

        pose.pose.orientation.w = pose_now.pose.orientation.w;
        pose.pose.orientation.x = pose_now.pose.orientation.x;
        pose.pose.orientation.y = pose_now.pose.orientation.y;
        pose.pose.orientation.z = pose_now.pose.orientation.z;

        // geometry_msgs::Twist vel;
        // vel.linear.x = 0;
        // vel.linear.y = 0;
        // vel.linear.z = 0;

        // local_vel_pub.publish()
        local_pos_pub.publish(pose);

        if (current_state.armed)
        {
            std::cout << "SETPT HERE" << std::endl;
            std::cout << "x: " << pose.pose.position.x << std::endl;
            std::cout << "y: " << pose.pose.position.y << std::endl;
            std::cout << "z: " << pose.pose.position.z << std::endl << std::endl;

            std::cout << "ex: " << pose.pose.position.x - pose_now.pose.position.x << std::endl;
            std::cout << "ey: " << pose.pose.position.y - pose_now.pose.position.y << std::endl;
            std::cout << "ez: " << pose.pose.position.z - pose_now.pose.position.z << std::endl;
        }

        ros::spinOnce();
        rate.sleep();
     }
 
     return 0;
 }