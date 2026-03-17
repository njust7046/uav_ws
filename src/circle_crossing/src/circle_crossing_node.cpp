/**
 * circle_crossing_node.cpp — 穿越圆圈障碍物飞行节点
 *
 * 功能：从 YAML 配置读取圆圈位置（各自独立高度），控制无人机依次穿越每个圆心，
 *       到达终点后爬升至安全高度，直线飞回起飞点降落。
 *
 * 每个圆圈前后插入过渡点，确保无人机以水平姿态穿越圆心。
 * 穿越圆圈时不悬停，连续飞行。
 *
 * 依赖链路：MID-360 → FAST_LIO(定位) → lidar_to_mavros(桥接) → MAVROS → PX4
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <vector>
#include <string>

struct Waypoint3D { double x, y, z; };

/* ======================== 全局变量 ======================== */

mavros_msgs::State current_state;
nav_msgs::Odometry local_pos;
tf::Quaternion quat;
double roll, pitch, yaw;
float init_x = 0, init_y = 0, init_z = 0;
bool init_recorded = false;

/* ======================== 回调函数 ======================== */

void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state = *msg; }

void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;
    if (!init_recorded && msg->pose.pose.position.z != 0)
    {
        init_x = msg->pose.pose.position.x;
        init_y = msg->pose.pose.position.y;
        init_z = msg->pose.pose.position.z;
        init_recorded = true;
        ROS_INFO("Home position recorded: x=%.2f y=%.2f z=%.2f", init_x, init_y, init_z);
    }
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

/* ======================== 工具函数 ======================== */

double distance_to(double tx, double ty, double tz)
{
    double dx = local_pos.pose.pose.position.x - tx;
    double dy = local_pos.pose.pose.position.y - ty;
    double dz = local_pos.pose.pose.position.z - tz;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

/* ======================== 主函数 ======================== */

int main(int argc, char **argv)
{
    ros::init(argc, argv, "circle_crossing_node");
    ros::NodeHandle nh("~");

    /* ---------- 加载参数 ---------- */
    double takeoff_height, safe_return_height, reach_threshold, approach_distance;
    double goal_x, goal_y;
    nh.param("/circle_crossing/takeoff_height",      takeoff_height,      1.0);
    nh.param("/circle_crossing/safe_return_height",   safe_return_height,  2.0);
    nh.param("/circle_crossing/reach_threshold",      reach_threshold,     0.3);
    nh.param("/circle_crossing/approach_distance",    approach_distance,   1.5);
    nh.param("/circle_crossing/goal_x",              goal_x,              12.0);
    nh.param("/circle_crossing/goal_y",              goal_y,              0.0);

    // 读取圆圈列表
    std::vector<Waypoint3D> circles;
    XmlRpc::XmlRpcValue c_list;
    if (nh.getParam("/circle_crossing/circles", c_list))
    {
        for (int i = 0; i < c_list.size(); i++)
        {
            Waypoint3D c;
            c.x = static_cast<double>(c_list[i]["x"]);
            c.y = static_cast<double>(c_list[i]["y"]);
            c.z = static_cast<double>(c_list[i]["z"]);
            circles.push_back(c);
        }
    }
    ROS_INFO("Loaded %lu circles, takeoff=%.1fm, safe_return=%.1fm",
             circles.size(), takeoff_height, safe_return_height);

    if (circles.empty())
    {
        ROS_ERROR("Circle list is empty, exiting");
        return 1;
    }
    /* ---------- 展开航点序列：过渡点 + 圆心 ---------- */
    std::vector<Waypoint3D> flight_path;

    for (size_t i = 0; i < circles.size(); i++)
    {
        // 计算飞向当前圆圈的方向向量
        double from_x, from_y;
        if (i == 0)
        {
            // 第一个圆圈：从起飞点出发
            from_x = 0.0;
            from_y = 0.0;
        }
        else
        {
            from_x = circles[i - 1].x;
            from_y = circles[i - 1].y;
        }

        double dx = circles[i].x - from_x;
        double dy = circles[i].y - from_y;
        double dist = std::sqrt(dx*dx + dy*dy);

        if (dist < 1e-6) dist = 1.0; // 防止除零
        double ux = dx / dist;  // 单位方向向量
        double uy = dy / dist;

        // 进入过渡点：圆心前方 approach_distance，高度与圆心相同
        Waypoint3D approach;
        approach.x = circles[i].x - ux * approach_distance;
        approach.y = circles[i].y - uy * approach_distance;
        approach.z = circles[i].z;
        flight_path.push_back(approach);

        // 圆心
        flight_path.push_back(circles[i]);

        // 离开过渡点：圆心后方 approach_distance，高度与圆心相同
        Waypoint3D depart;
        depart.x = circles[i].x + ux * approach_distance;
        depart.y = circles[i].y + uy * approach_distance;
        depart.z = circles[i].z;
        flight_path.push_back(depart);
    }

    ROS_INFO("Flight path expanded to %lu waypoints (with transition points)",
             flight_path.size());
    for (size_t i = 0; i < flight_path.size(); i++)
    {
        ROS_INFO("  WP[%lu]: x=%.2f y=%.2f z=%.2f", i,
                 flight_path[i].x, flight_path[i].y, flight_path[i].z);
    }
    /* ---------- 初始化 ROS 通信接口 ---------- */
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", 10, local_pos_cb);
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "/mavros/cmd/arming");
    ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "/mavros/set_mode");

    ros::Rate rate(20.0);

    /* ---------- 等待飞控连接 ---------- */
    ROS_INFO("Waiting for FCU connection...");
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    ROS_INFO("Waiting for FAST_LIO initialization...");
    while (ros::ok() && !init_recorded)
    {
        ros::spinOnce();
        rate.sleep();
    }

    /* ---------- 预发 setpoint (5s) ---------- */
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x = init_x;
    pose.pose.position.y = init_y;
    pose.pose.position.z = init_z + takeoff_height;

    ROS_INFO("Streaming setpoints before OFFBOARD request...");
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        pose.header.stamp = ros::Time::now();
        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    /* ---------- 切 OFFBOARD + 解锁 + 起飞 ---------- */
    mavros_msgs::SetMode offb_mode;
    offb_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    ROS_INFO("Requesting OFFBOARD mode and arming...");
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (mode_client.call(offb_mode) && offb_mode.response.mode_sent)
                ROS_INFO("OFFBOARD mode enabled");
            last_request = ros::Time::now();
        }
        else if (!current_state.armed &&
                 (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                ROS_INFO("Vehicle armed");
            last_request = ros::Time::now();
        }

        if (current_state.armed && current_state.mode == "OFFBOARD" &&
            fabs(local_pos.pose.pose.position.z - init_z - takeoff_height) < reach_threshold)
        {
            ROS_INFO("Reached takeoff height %.2fm", takeoff_height);
            break;
        }

        pose.header.stamp = ros::Time::now();
        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    /* ---------- 依次穿越圆圈（不悬停） ---------- */
    for (size_t i = 0; i < flight_path.size() && ros::ok(); i++)
    {
        double target_x = init_x + flight_path[i].x;
        double target_y = init_y + flight_path[i].y;
        double target_z = init_z + flight_path[i].z;

        const char* wp_type;
        if (i % 3 == 0)      wp_type = "APPROACH";
        else if (i % 3 == 1) wp_type = "CIRCLE";
        else                  wp_type = "DEPART";

        ROS_INFO("Flying to WP[%lu/%lu] (%s): x=%.2f y=%.2f z=%.2f",
                 i + 1, flight_path.size(), wp_type, target_x, target_y, target_z);

        pose.pose.position.x = target_x;
        pose.pose.position.y = target_y;
        pose.pose.position.z = target_z;

        while (ros::ok())
        {
            if (distance_to(target_x, target_y, target_z) < reach_threshold)
            {
                ROS_INFO("Reached WP[%lu] (%s)", i + 1, wp_type);
                break;
            }
            pose.header.stamp = ros::Time::now();
            pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
        // 穿越圆圈时不悬停，立即飞向下一个航点
    }

    /* ---------- 飞到终点 ---------- */
    double goal_abs_x = init_x + goal_x;
    double goal_abs_y = init_y + goal_y;
    double goal_abs_z = init_z + circles.back().z; // 终点高度与最后一个圆圈相同

    ROS_INFO("Heading to goal: x=%.2f y=%.2f z=%.2f", goal_abs_x, goal_abs_y, goal_abs_z);
    pose.pose.position.x = goal_abs_x;
    pose.pose.position.y = goal_abs_y;
    pose.pose.position.z = goal_abs_z;

    while (ros::ok())
    {
        if (distance_to(goal_abs_x, goal_abs_y, goal_abs_z) < reach_threshold)
        {
            ROS_INFO("Reached goal point");
            break;
        }
        pose.header.stamp = ros::Time::now();
        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }
    /* ---------- 爬升到安全高度 ---------- */
    double safe_z = init_z + safe_return_height;
    ROS_INFO("Climbing to safe return height %.2fm...", safe_return_height);
    pose.pose.position.x = goal_abs_x;
    pose.pose.position.y = goal_abs_y;
    pose.pose.position.z = safe_z;

    while (ros::ok())
    {
        if (distance_to(goal_abs_x, goal_abs_y, safe_z) < reach_threshold)
        {
            ROS_INFO("Reached safe return height");
            break;
        }
        pose.header.stamp = ros::Time::now();
        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    /* ---------- 直线飞回起飞点上方 ---------- */
    ROS_INFO("Returning to home position...");
    pose.pose.position.x = init_x;
    pose.pose.position.y = init_y;
    pose.pose.position.z = safe_z;

    while (ros::ok())
    {
        if (distance_to(init_x, init_y, safe_z) < reach_threshold)
        {
            ROS_INFO("Arrived above home position");
            break;
        }
        pose.header.stamp = ros::Time::now();
        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    /* ---------- 自动降落 ---------- */
    ROS_INFO("Landing...");
    mavros_msgs::SetMode land_mode;
    land_mode.request.custom_mode = "AUTO.LAND";

    while (ros::ok())
    {
        if (current_state.mode != "AUTO.LAND" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (mode_client.call(land_mode) && land_mode.response.mode_sent)
                ROS_INFO("AUTO.LAND mode enabled");
            last_request = ros::Time::now();
        }

        if (!current_state.armed)
        {
            ROS_INFO("Landing complete, mission finished");
            break;
        }

        pose.header.stamp = ros::Time::now();
        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
