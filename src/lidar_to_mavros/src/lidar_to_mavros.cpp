// lidar_to_mavros: 将 FAST_LIO 里程计桥接到 MAVROS 和 px4ctrl
//
// 订阅: /Odometry (nav_msgs/Odometry) — FAST_LIO 输出
// 发布: /mavros/vision_pose/pose (geometry_msgs/PoseStamped) — 给 PX4 外部定位
//       /Odom_high_freq (nav_msgs/Odometry) — 给 px4ctrl 高频里程计

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher vision_pose_pub;
ros::Publisher odom_high_freq_pub;

void odom_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    // 转发给 mavros vision_pose
    geometry_msgs::PoseStamped pose;
    pose.header = msg->header;
    pose.pose   = msg->pose.pose;
    vision_pose_pub.publish(pose);

    // 转发高频 odom 给 px4ctrl
    odom_high_freq_pub.publish(*msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_to_mavros");
    ros::NodeHandle nh;

    vision_pose_pub    = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose", 100);
    odom_high_freq_pub = nh.advertise<nav_msgs::Odometry>("/Odom_high_freq", 100);

    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/Odometry", 100, odom_cb,
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay());

    ROS_INFO("lidar_to_mavros started");
    ros::spin();
    return 0;
}
