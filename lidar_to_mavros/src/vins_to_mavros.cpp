#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std;


class vins_bridge
{
public:
    vins_bridge(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_);
    double pi;

    struct attitude { double pitch; double roll; double yaw; };

    attitude estimatedAttitude;
    attitude px4Attitude;

    geometry_msgs::PoseStamped px4Pose;
    geometry_msgs::PoseStamped estimatedPose;

    bool estimatedOdomRec_flag;

    ros::Rate *rate;
    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    ros::Subscriber px4Pose_sub;
    ros::Publisher vision_pose_pub;
    ros::Subscriber odom_sub;

    void px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void vins_odom_cb(const nav_msgs::Odometry::ConstPtr &msg);
    void start();
};

vins_bridge::vins_bridge(const ros::NodeHandle &nh_, const ros::NodeHandle &nh_private_) : nh(nh_), nh_private(nh_private_)
{
    pi = 3.1415926;
    rate = new ros::Rate(30);

    std::string vins_odom_topic;
    nh_private.param<std::string>("vins_odom_topic", vins_odom_topic, "/vins_estimator/odometry");
    ROS_INFO("vins_to_mavros: vins_odom_topic = %s (pure forwarding, no position compensation)",
             vins_odom_topic.c_str());

    px4Pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &vins_bridge::px4Pose_cb, this);
    odom_sub = nh.subscribe<nav_msgs::Odometry>(vins_odom_topic, 2, &vins_bridge::vins_odom_cb, this);
    vision_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/vision_pose/pose", 10);

    estimatedOdomRec_flag = false;
    estimatedAttitude.pitch = estimatedAttitude.roll = estimatedAttitude.yaw = 0;
    px4Attitude.pitch = px4Attitude.roll = px4Attitude.yaw = 0;
}

void vins_bridge::vins_odom_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    // 纯转发：只取位姿与时间戳，不做任何修正
    estimatedPose.pose = msg->pose.pose;
    estimatedPose.header.stamp = msg->header.stamp;

    // 四元数转 RPY（仅用于调试显示）
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    estimatedAttitude.pitch = pitch * 180 / pi;
    estimatedAttitude.roll = roll * 180 / pi;
    estimatedAttitude.yaw = yaw * 180 / pi;

    estimatedOdomRec_flag = true;
}

void vins_bridge::px4Pose_cb(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    px4Pose.pose = msg->pose;
    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.orientation, quat);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    px4Attitude.pitch = pitch * 180 / pi;
    px4Attitude.roll = roll * 180 / pi;
    px4Attitude.yaw = yaw * 180 / pi;
}

void vins_bridge::start()
{
    while (ros::ok())
    {
        if (estimatedOdomRec_flag == false)
        {
            cout << "\033[K"
                 << "\033[31m vinsPose no receive!!!  Waiting for pose\033[0m" << endl;
        }
        else
        {
            vision_pose_pub.publish(estimatedPose);

            cout << "\033[K"
                 << "\033[32m vins estimate ok !\033[0m" << endl;
            cout << "\033[K"
                 << "       VINS-Pose               Px4-Pose" << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "x      " << estimatedPose.pose.position.x << "\t\t" << px4Pose.pose.position.x << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "y      " << estimatedPose.pose.position.y << "\t\t" << px4Pose.pose.position.y << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "z      " << estimatedPose.pose.position.z << "\t\t" << px4Pose.pose.position.z << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "pitch  " << estimatedAttitude.pitch << "\t\t" << px4Attitude.pitch << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "roll   " << estimatedAttitude.roll << "\t\t" << px4Attitude.roll << endl;
            cout << setiosflags(ios::fixed) << setprecision(7)
                 << "\033[K"
                 << "yaw    " << estimatedAttitude.yaw << "\t\t" << px4Attitude.yaw << endl;
            cout << "\033[9A" << endl;
        }
        ros::spinOnce();
        rate->sleep();
    }
    cout << "\033[9B" << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vins_to_mavros");
    ros::NodeHandle nh_("");
    ros::NodeHandle nh_private_("~");

    vins_bridge bridge(nh_, nh_private_);
    bridge.start();

    return 0;
}
