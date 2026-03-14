/**
 * inspection_node.cpp — 室内定点覆盖巡检飞行节点
 *
 * 功能：从 YAML 配置文件读取航点列表，控制无人机依次飞往各航点，
 *       完成后自动返回起飞点并降落。
 *
 * 依赖链路：MID-360 → FAST_LIO(定位) → lidar_to_mavros(桥接) → MAVROS → PX4
 *
 * 发布话题：/mavros/setpoint_position/local  (20Hz 目标位置)
 * 订阅话题：/mavros/state                    (飞控状态：模式、解锁等)
 *           /mavros/local_position/odom       (当前位置里程计)
 * 调用服务：/mavros/cmd/arming               (解锁/上锁)
 *           /mavros/set_mode                  (切换飞行模式)
 *
 * 状态机流程：
 *   等待FCU连接 → 等待FAST_LIO定位初始化 → 预发setpoint(5s)
 *   → 切OFFBOARD模式 → 解锁 → 起飞爬升
 *   → 逐航点飞行+悬停 → 返回起飞点 → AUTO.LAND降落
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>  // 目标位置消息类型
#include <mavros_msgs/CommandBool.h>    // 解锁/上锁服务类型
#include <mavros_msgs/SetMode.h>        // 模式切换服务类型
#include <mavros_msgs/State.h>          // 飞控状态消息类型
#include <nav_msgs/Odometry.h>          // 里程计消息类型（位置+姿态）
#include <tf/transform_listener.h>      // 四元数→欧拉角转换工具
#include <cmath>
#include <vector>
#include <string>

/**
 * 航点结构体
 * x, y 为相对起飞点的水平偏移量（单位：米）
 * 高度固定为 flight_height，不在航点中指定
 */
struct Waypoint { double x, y; };

/* ======================== 全局变量 ======================== */

// 飞控当前状态：connected（是否连接）、armed（是否解锁）、mode（当前模式）
mavros_msgs::State current_state;

// 当前位置里程计，由 FAST_LIO 定位经 lidar_to_mavros 桥接后通过 MAVROS 输出
nav_msgs::Odometry local_pos;

tf::Quaternion quat;         // 当前姿态四元数（从里程计消息中提取）
double roll, pitch, yaw;     // 由四元数转换得到的欧拉角（单位：弧度）

// 起飞原点坐标，在节点启动后首次收到有效里程计时记录
// 所有航点坐标均为相对此原点的偏移量
float init_x = 0, init_y = 0, init_z = 0;
bool init_recorded = false;  // 标志位：起飞原点是否已记录

/* ======================== 回调函数 ======================== */

/**
 * 飞控状态回调
 * 订阅 /mavros/state，实时更新 current_state
 * current_state.connected : MAVROS 与 PX4 的 MAVLink 连接是否建立
 * current_state.armed      : 电机是否已解锁
 * current_state.mode       : 当前飞行模式（如 "OFFBOARD"、"AUTO.LAND" 等）
 */
void state_cb(const mavros_msgs::State::ConstPtr& msg) { current_state = *msg; }

/**
 * 里程计回调
 * 订阅 /mavros/local_position/odom，实时更新当前位置和姿态
 *
 * 起飞原点记录逻辑：
 *   FAST_LIO 初始化完成前会输出全零位置，用 z != 0 判断初始化是否完成。
 *   首次收到非零 z 值时将当前位置记为起飞原点 (init_x, init_y, init_z)。
 *   此后 init_recorded = true，不再重复记录。
 */
void local_pos_cb(const nav_msgs::Odometry::ConstPtr& msg)
{
    local_pos = *msg;

    // 首次收到有效位置（z != 0）时记录为起飞原点
    if (!init_recorded && msg->pose.pose.position.z != 0)
    {
        init_x = msg->pose.pose.position.x;
        init_y = msg->pose.pose.position.y;
        init_z = msg->pose.pose.position.z;
        init_recorded = true;
        ROS_INFO("Home position recorded: x=%.2f y=%.2f z=%.2f", init_x, init_y, init_z);
    }

    // 将姿态四元数转换为欧拉角，yaw 可用于后续偏航控制（当前未使用）
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

/* ======================== 工具函数 ======================== */

/**
 * 计算当前位置到目标点 (tx, ty, tz) 的三维欧氏距离
 * 用于判断是否到达航点：distance_to(target) < reach_threshold 即视为到达
 */
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
    ros::init(argc, argv, "inspection_node");
    // 使用私有命名空间 "~"，但参数读取时用绝对路径 /indoor_inspection/...
    ros::NodeHandle nh("~");

    /* ---------- 第1步：从参数服务器加载配置 ----------
     * waypoints.yaml 由 inspection.launch 中的 <rosparam> 标签加载到
     * /indoor_inspection/ 命名空间，这里用绝对路径读取。
     * 第三个参数为默认值，参数不存在时使用。
     */
    double flight_height, reach_threshold, hover_time;
    nh.param("/indoor_inspection/flight_height",   flight_height,   1.2);  // 巡检飞行高度(m)
    nh.param("/indoor_inspection/reach_threshold", reach_threshold, 0.3);  // 到达判定距离阈值(m)
    nh.param("/indoor_inspection/hover_time",      hover_time,      3.0);  // 每个航点悬停时间(s)

    // 读取航点列表，YAML 中为列表格式，ROS 参数服务器以 XmlRpcValue 类型存储
    std::vector<Waypoint> waypoints;
    XmlRpc::XmlRpcValue wp_list;
    if (nh.getParam("/indoor_inspection/waypoints", wp_list))
    {
        for (int i = 0; i < wp_list.size(); i++)
        {
            Waypoint wp;
            wp.x = static_cast<double>(wp_list[i]["x"]);  // 相对起飞点的东向偏移(m)
            wp.y = static_cast<double>(wp_list[i]["y"]);  // 相对起飞点的北向偏移(m)
            waypoints.push_back(wp);
        }
    }
    ROS_INFO("Loaded %lu waypoints, height=%.1fm, threshold=%.2fm, hover=%.1fs",
             waypoints.size(), flight_height, reach_threshold, hover_time);

    // 航点为空则无法执行任务，直接退出
    if (waypoints.empty())
    {
        ROS_ERROR("Waypoint list is empty, exiting");
        return 1;
    }

    /* ---------- 第2步：初始化 ROS 通信接口 ---------- */

    // 订阅飞控状态，队列深度 10，回调更新 current_state
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, state_cb);

    // 订阅里程计，队列深度 10，回调更新 local_pos 并记录起飞原点
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", 10, local_pos_cb);

    // 发布目标位置，PX4 在 OFFBOARD 模式下会跟踪此话题的位置指令
    ros::Publisher pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);

    // 解锁服务：发送 true 解锁电机，false 上锁
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "/mavros/cmd/arming");

    // 模式切换服务：切换 PX4 飞行模式（OFFBOARD / AUTO.LAND 等）
    ros::ServiceClient mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "/mavros/set_mode");

    // 控制循环频率 20Hz
    // PX4 要求 OFFBOARD setpoint 发布频率 ≥ 2Hz，否则自动退出 OFFBOARD 模式
    ros::Rate rate(20.0);

    /* ---------- 第3步：等待飞控连接 ----------
     * MAVROS 节点启动后需要通过串口与 PX4 建立 MAVLink 握手，
     * connected 变为 true 后才能发送指令。
     */
    ROS_INFO("Waiting for FCU connection...");
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();  // 处理回调，更新 current_state.connected
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    // 等待 FAST_LIO 完成初始化并输出有效位置，init_recorded 由回调置位
    ROS_INFO("Waiting for FAST_LIO initialization...");
    while (ros::ok() && !init_recorded)
    {
        ros::spinOnce();
        rate.sleep();
    }

    /* ---------- 第4步：预发 setpoint ----------
     * PX4 硬性要求：在请求切入 OFFBOARD 模式之前，
     * setpoint 话题必须已经有持续的数据流，否则切换请求会被拒绝。
     * 这里以 20Hz 发布 100 次（约 5 秒），目标位置为起飞点正上方。
     */
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "odom";           // 坐标系：里程计坐标系
    pose.pose.position.x = init_x;           // 水平位置保持在起飞点
    pose.pose.position.y = init_y;
    pose.pose.position.z = init_z + flight_height;  // 目标高度 = 起飞点高度 + 飞行高度

    ROS_INFO("Streaming setpoints before OFFBOARD request...");
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        pose.header.stamp = ros::Time::now();  // 每次发布前更新时间戳，避免 PX4 认为数据过期
        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    /* ---------- 第5步：切 OFFBOARD + 解锁 + 起飞 ----------
     * 请求顺序：先切 OFFBOARD，成功后再解锁。
     * 每次服务调用间隔 5 秒，防止频繁请求。
     * 循环内持续发布 setpoint，维持 OFFBOARD 模式不退出。
     * 判断起飞完成：已解锁 + 已在 OFFBOARD + 高度误差 < reach_threshold。
     */
    mavros_msgs::SetMode offb_mode;
    offb_mode.request.custom_mode = "OFFBOARD";  // PX4 外部控制模式

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;  // true = 解锁

    ros::Time last_request = ros::Time::now();

    ROS_INFO("Requesting OFFBOARD mode and arming...");
    while (ros::ok())
    {
        // 当前不在 OFFBOARD 且距上次请求已超过 5 秒 → 请求切入 OFFBOARD
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (mode_client.call(offb_mode) && offb_mode.response.mode_sent)
                ROS_INFO("OFFBOARD mode enabled");
            last_request = ros::Time::now();
        }
        // 已在 OFFBOARD 但未解锁，且距上次请求已超过 5 秒 → 请求解锁
        else if (!current_state.armed &&
                 (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                ROS_INFO("Vehicle armed");
            last_request = ros::Time::now();
        }

        // 起飞完成判断：已解锁 + 已在 OFFBOARD + 当前高度与目标高度误差 < 阈值
        if (current_state.armed && current_state.mode == "OFFBOARD" &&
            fabs(local_pos.pose.pose.position.z - init_z - flight_height) < reach_threshold)
        {
            ROS_INFO("Reached takeoff height %.2fm", flight_height);
            break;
        }

        // 持续发布目标位置，维持 OFFBOARD 模式（PX4 超时会自动退出 OFFBOARD）
        pose.header.stamp = ros::Time::now();
        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    /* ---------- 第6步：依次飞往每个航点 ----------
     * 对每个航点执行：
     *   1) 计算绝对坐标 = 起飞原点坐标 + YAML 中配置的相对偏移
     *   2) 持续发布目标位置，直到三维欧氏距离 < reach_threshold
     *   3) 到达后在该位置悬停 hover_time 秒（持续发布同一位置以保持定点）
     */
    for (size_t i = 0; i < waypoints.size() && ros::ok(); i++)
    {
        // 将相对偏移转换为绝对坐标
        double target_x = init_x + waypoints[i].x;
        double target_y = init_y + waypoints[i].y;
        double target_z = init_z + flight_height;  // 所有航点保持相同飞行高度

        ROS_INFO("Heading to waypoint %lu/%lu: x=%.2f y=%.2f z=%.2f",
                 i + 1, waypoints.size(), target_x, target_y, target_z);

        pose.pose.position.x = target_x;
        pose.pose.position.y = target_y;
        pose.pose.position.z = target_z;

        // 飞向航点：持续发布目标位置，直到到达
        while (ros::ok())
        {
            if (distance_to(target_x, target_y, target_z) < reach_threshold)
            {
                ROS_INFO("Reached waypoint %lu", i + 1);
                break;
            }
            pose.header.stamp = ros::Time::now();
            pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        // 悬停：在航点位置持续发布 hover_time 秒，保持定点不漂移
        ROS_INFO("Hovering for %.1f seconds...", hover_time);
        ros::Time hover_start = ros::Time::now();
        while (ros::ok() && (ros::Time::now() - hover_start < ros::Duration(hover_time)))
        {
            pose.header.stamp = ros::Time::now();
            pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
    }

    /* ---------- 第7步：返回起飞点上方 ----------
     * 所有航点完成后，飞回起飞原点正上方（保持飞行高度），为降落做准备。
     * 同样用欧氏距离判断是否到达。
     */
    ROS_INFO("All waypoints completed, returning to home");
    pose.pose.position.x = init_x;
    pose.pose.position.y = init_y;
    pose.pose.position.z = init_z + flight_height;

    while (ros::ok())
    {
        if (distance_to(init_x, init_y, init_z + flight_height) < reach_threshold)
        {
            ROS_INFO("Arrived above home position");
            break;
        }
        pose.header.stamp = ros::Time::now();
        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    /* ---------- 第8步：自动降落 ----------
     * 切换到 PX4 原生的 AUTO.LAND 模式，由飞控接管降落全程。
     * AUTO.LAND 内置触地检测和自动上锁，比手动控制 z 轴下降更安全可靠。
     *
     * 降落完成判断：PX4 落地后会自动上锁（由参数 COM_DISARM_LAND 控制延时），
     * current_state.armed 变为 false 即表示任务全部完成。
     */
    ROS_INFO("Landing...");
    mavros_msgs::SetMode land_mode;
    land_mode.request.custom_mode = "AUTO.LAND";

    while (ros::ok())
    {
        // 当前不在 AUTO.LAND 且距上次请求超过 5 秒 → 请求切入降落模式
        if (current_state.mode != "AUTO.LAND" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (mode_client.call(land_mode) && land_mode.response.mode_sent)
                ROS_INFO("AUTO.LAND mode enabled");
            last_request = ros::Time::now();
        }

        // 落地并自动上锁后退出循环，节点正常结束
        if (!current_state.armed)
        {
            ROS_INFO("Landing complete, mission finished");
            break;
        }

        // AUTO.LAND 期间继续发布 setpoint，防止 MAVROS 报警（实际由飞控接管）
        pose.header.stamp = ros::Time::now();
        pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
