/**
 * avoidance_node.cpp — VFH+ 室外避障演示节点
 *
 * 功能：无人机起飞后使用 VFH+ 算法避障飞向目标点，到达后自动降落。
 *
 * 算法流程（20Hz）：
 *   1. 从 /cloud_registered 点云中切取飞行高度 ±0.5m 的水平切片
 *   2. 构建 72 扇区（每个 5°）的极坐标障碍物直方图
 *   3. 二值化：点数 > 阈值的扇区标记为"阻塞"
 *   4. 找出所有连续空闲扇区（gap），处理 0°/360° 环绕
 *   5. VFH+ 代价函数选择最优方向
 *   6. 沿选定方向生成步进 setpoint
 *
 * 状态机：
 *   等待FCU → 等待FAST_LIO → 预发setpoint(5s) → OFFBOARD+解锁 → 起飞
 *   → VFH+避障飞向目标 → 到达目标 → 悬停(2s) → AUTO.LAND
 *
 * 依赖链路：MID-360 → FAST_LIO(定位+点云) → MAVROS → PX4
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>

/* ======================== 全局变量 ======================== */

mavros_msgs::State current_state;
nav_msgs::Odometry local_pos;
tf::Quaternion quat;
double roll, pitch, yaw;

float init_x = 0, init_y = 0, init_z = 0;
bool init_recorded = false;

// 最新点云（由回调更新）
pcl::PointCloud<pcl::PointXYZ>::Ptr latest_cloud(new pcl::PointCloud<pcl::PointXYZ>);
bool cloud_received = false;
ros::Time cloud_stamp;  // 点云接收时间戳，用于时效性检查

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

void cloud_cb(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *temp);
    latest_cloud = temp;
    cloud_received = true;
    cloud_stamp = ros::Time::now();
}

/* ======================== 工具函数 ======================== */

double distance_to(double tx, double ty, double tz)
{
    double dx = local_pos.pose.pose.position.x - tx;
    double dy = local_pos.pose.pose.position.y - ty;
    double dz = local_pos.pose.pose.position.z - tz;
    return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double distance_2d(double tx, double ty)
{
    double dx = local_pos.pose.pose.position.x - tx;
    double dy = local_pos.pose.pose.position.y - ty;
    return std::sqrt(dx*dx + dy*dy);
}

// 将角度归一化到 [-pi, pi]
double normalize_angle(double a)
{
    while (a > M_PI)  a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
}

// 根据航向角生成四元数（仅 yaw，roll/pitch 为 0）
geometry_msgs::Quaternion yaw_to_quat(double yaw_rad)
{
    tf::Quaternion q;
    q.setRPY(0, 0, yaw_rad);
    geometry_msgs::Quaternion msg;
    tf::quaternionTFToMsg(q, msg);
    return msg;
}

/* ======================== VFH+ 核心算法 ======================== */

/**
 * VFH+ 避障方向选择（优化版）
 *
 * 优化点：
 *   - 多层点云切片（低/中/高），检测不同高度的障碍物
 *   - 直方图膨胀，给障碍物增加角度安全边界
 *
 * @param cloud          当前点云
 * @param cur_z          当前飞行高度
 * @param slice_half     切片半高度
 * @param max_range      检测最大距离
 * @param num_sectors    扇区数
 * @param obs_thresh     阻塞阈值（点数）
 * @param enlarge        膨胀扇区数
 * @param multi_layer    是否启用多层切片
 * @param layer_offset   额外层偏移
 * @param goal_angle     目标方向角(rad)
 * @param cur_yaw        当前航向(rad)
 * @param prev_angle     上一帧选择方向(rad)
 * @param w_target       目标方向权重
 * @param w_smooth       航向平滑权重
 * @param w_prev         方向连续性权重
 * @param min_dist       [输出] 最近障碍物距离
 * @return               最优前进方向角(rad)，无空闲扇区时返回 NAN
 */
double vfh_select_direction(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
    double cur_z, double slice_half, double max_range,
    int num_sectors, int obs_thresh, int enlarge,
    bool multi_layer, double layer_offset,
    double ground_z, double ground_clearance,
    double goal_angle, double cur_yaw, double prev_angle,
    double w_target, double w_smooth, double w_prev,
    double& min_dist)
{
    double sector_width = 2.0 * M_PI / num_sectors;
    std::vector<int> histogram(num_sectors, 0);
    min_dist = std::numeric_limits<double>::max();

    double cur_x = local_pos.pose.pose.position.x;
    double cur_y = local_pos.pose.pose.position.y;

    // 地面高度阈值：低于此高度的点视为地面，不计入直方图
    double z_floor = ground_z + ground_clearance;

    // 构建切片层：中层必有，多层模式额外加低层和高层
    struct Slice { double z_min; double z_max; };
    std::vector<Slice> slices;
    slices.push_back({std::max(cur_z - slice_half, z_floor), cur_z + slice_half});
    if (multi_layer)
    {
        double low_min = std::max(cur_z - layer_offset - slice_half, z_floor);
        double low_max = cur_z - layer_offset + slice_half;
        if (low_max > z_floor)
            slices.push_back({low_min, low_max});
        slices.push_back({cur_z + layer_offset - slice_half, cur_z + layer_offset + slice_half});
    }

    // 步骤1：多层切片 + 构建极坐标直方图
    for (size_t i = 0; i < cloud->points.size(); i++)
    {
        const pcl::PointXYZ& pt = cloud->points[i];

        // 检查点是否落在任一切片层内
        bool in_slice = false;
        for (size_t s = 0; s < slices.size(); s++)
        {
            if (pt.z >= slices[s].z_min && pt.z <= slices[s].z_max)
            { in_slice = true; break; }
        }
        if (!in_slice) continue;

        double dx = pt.x - cur_x;
        double dy = pt.y - cur_y;
        double dist = std::sqrt(dx*dx + dy*dy);
        if (dist > max_range || dist < 0.1) continue;

        if (dist < min_dist) min_dist = dist;

        double angle = std::atan2(dy, dx);
        double a_pos = angle < 0 ? angle + 2.0 * M_PI : angle;
        int idx = static_cast<int>(a_pos / sector_width) % num_sectors;
        histogram[idx]++;
    }

    // 步骤2：二值化
    std::vector<bool> blocked(num_sectors, false);
    for (int i = 0; i < num_sectors; i++)
        blocked[i] = (histogram[i] > obs_thresh);

    // 步骤2.5：直方图膨胀 — 阻塞扇区向两侧各扩展 enlarge 个扇区
    if (enlarge > 0)
    {
        std::vector<bool> expanded = blocked;
        for (int i = 0; i < num_sectors; i++)
        {
            if (blocked[i])
            {
                for (int e = 1; e <= enlarge; e++)
                {
                    expanded[(i + e) % num_sectors] = true;
                    expanded[(i - e + num_sectors) % num_sectors] = true;
                }
            }
        }
        blocked = expanded;
    }

    // 步骤3：找出所有连续空闲扇区（gap），处理环绕
    struct Gap { int start; int end; int width; };
    std::vector<Gap> gaps;

    // 找到第一个阻塞扇区作为扫描起点，避免环绕 gap 被拆成两段
    int scan_start = -1;
    for (int i = 0; i < num_sectors; i++)
    {
        if (blocked[i]) { scan_start = i; break; }
    }

    if (scan_start == -1)
    {
        // 全部空闲，直接朝目标方向走
        return goal_angle;
    }

    int gap_start = -1;
    for (int k = 0; k < num_sectors; k++)
    {
        int i = (scan_start + k) % num_sectors;
        if (!blocked[i])
        {
            if (gap_start == -1) gap_start = i;
        }
        else
        {
            if (gap_start != -1)
            {
                // 计算 gap 宽度（考虑环绕）
                int prev = (i - 1 + num_sectors) % num_sectors;
                int w = (prev - gap_start + num_sectors) % num_sectors + 1;
                gaps.push_back({gap_start, prev, w});
                gap_start = -1;
            }
        }
    }
    // 处理扫描结束时仍在 gap 中的情况
    if (gap_start != -1)
    {
        int prev = (scan_start - 1 + num_sectors) % num_sectors;
        int w = (prev - gap_start + num_sectors) % num_sectors + 1;
        gaps.push_back({gap_start, prev, w});
    }

    if (gaps.empty())
    {
        // 全部阻塞，返回 NAN 触发悬停
        return NAN;
    }

    // 步骤4：VFH+ 代价函数选择最优方向
    double best_angle = NAN;
    double best_cost = std::numeric_limits<double>::max();

    for (size_t g = 0; g < gaps.size(); g++)
    {
        // 对每个 gap，评估其中心方向和靠近目标的边缘方向
        std::vector<double> candidates;

        // gap 中心
        int center_idx = (gaps[g].start + gaps[g].width / 2) % num_sectors;
        candidates.push_back(center_idx * sector_width);

        // gap 两侧边缘（向内缩一个扇区，留安全余量）
        if (gaps[g].width > 2)
        {
            int left = (gaps[g].start + 1) % num_sectors;
            int right = (gaps[g].end - 1 + num_sectors) % num_sectors;
            candidates.push_back(left * sector_width);
            candidates.push_back(right * sector_width);
        }

        for (size_t c = 0; c < candidates.size(); c++)
        {
            double cand = candidates[c];
            // 代价 = 目标偏差 + 航向平滑 + 方向连续性
            double d_target = std::fabs(normalize_angle(cand - goal_angle));
            double d_smooth = std::fabs(normalize_angle(cand - cur_yaw));
            double d_prev   = std::fabs(normalize_angle(cand - prev_angle));
            double cost = w_target * d_target + w_smooth * d_smooth + w_prev * d_prev;

            if (cost < best_cost)
            {
                best_cost = cost;
                best_angle = cand;
            }
        }
    }

    return best_angle;
}

/* ======================== 主函数 ======================== */

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoidance_node");
    ros::NodeHandle nh("~");

    // --- 步骤1：加载参数 ---
    double flight_height, reach_threshold, hover_time;
    double goal_x, goal_y;
    double max_range, safety_radius, slowdown_radius, slice_half_height, step_distance;
    int num_sectors, obstacle_threshold, enlarge_sectors;
    double w_target, w_smooth, w_prev;
    bool multi_layer;
    double layer_offset, direction_alpha, stuck_timeout, retreat_distance;
    double ground_clearance, cloud_timeout;

    nh.param("/obstacle_avoidance/flight_height", flight_height, 2.0);
    nh.param("/obstacle_avoidance/reach_threshold", reach_threshold, 0.5);
    nh.param("/obstacle_avoidance/hover_time", hover_time, 2.0);
    nh.param("/obstacle_avoidance/goal_x", goal_x, 10.0);
    nh.param("/obstacle_avoidance/goal_y", goal_y, 0.0);
    nh.param("/obstacle_avoidance/max_range", max_range, 6.0);
    nh.param("/obstacle_avoidance/safety_radius", safety_radius, 1.0);
    nh.param("/obstacle_avoidance/slowdown_radius", slowdown_radius, 3.0);
    nh.param("/obstacle_avoidance/slice_half_height", slice_half_height, 0.5);
    nh.param("/obstacle_avoidance/step_distance", step_distance, 1.5);
    nh.param("/obstacle_avoidance/num_sectors", num_sectors, 72);
    nh.param("/obstacle_avoidance/obstacle_threshold", obstacle_threshold, 3);
    nh.param("/obstacle_avoidance/enlarge_sectors", enlarge_sectors, 2);
    nh.param("/obstacle_avoidance/multi_layer", multi_layer, true);
    nh.param("/obstacle_avoidance/layer_offset", layer_offset, 0.5);
    nh.param("/obstacle_avoidance/ground_clearance", ground_clearance, 0.3);
    nh.param("/obstacle_avoidance/direction_alpha", direction_alpha, 0.3);
    nh.param("/obstacle_avoidance/stuck_timeout", stuck_timeout, 3.0);
    nh.param("/obstacle_avoidance/retreat_distance", retreat_distance, 2.0);
    nh.param("/obstacle_avoidance/cloud_timeout", cloud_timeout, 1.0);
    nh.param("/obstacle_avoidance/w_target", w_target, 5.0);
    nh.param("/obstacle_avoidance/w_smooth", w_smooth, 2.0);
    nh.param("/obstacle_avoidance/w_prev", w_prev, 2.0);

    ROS_INFO("=== VFH+ Avoidance Demo (Optimized) ===");
    ROS_INFO("Goal: (%.1f, %.1f), Height: %.1f", goal_x, goal_y, flight_height);
    ROS_INFO("Max range: %.1f, Safety: %.1f, Slowdown: %.1f, Sectors: %d",
             max_range, safety_radius, slowdown_radius, num_sectors);
    ROS_INFO("Multi-layer: %s, Enlarge: %d, Alpha: %.2f",
             multi_layer ? "ON" : "OFF", enlarge_sectors, direction_alpha);

    // --- 步骤2：初始化 ROS 接口 ---
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "/mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/mavros/local_position/odom", 10, local_pos_cb);
    ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(
        "/cloud_registered", 1, cloud_cb);

    ros::Publisher setpoint_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/mavros/setpoint_position/local", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "/mavros/set_mode");

    ros::Rate rate(20.0);

    // --- 步骤3：等待 FCU 连接 ---
    ROS_INFO("Waiting for FCU connection...");
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected.");

    // --- 步骤4：等待 FAST_LIO 初始化 ---
    ROS_INFO("Waiting for FAST_LIO initialization...");
    while (ros::ok() && !init_recorded)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FAST_LIO initialized.");

    // 计算绝对目标点
    double abs_goal_x = init_x + goal_x;
    double abs_goal_y = init_y + goal_y;
    double abs_goal_z = init_z + flight_height;

    // 起飞目标
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "odom";
    pose.pose.position.x = init_x;
    pose.pose.position.y = init_y;
    pose.pose.position.z = abs_goal_z;
    pose.pose.orientation.w = 1.0;

    // --- 步骤5：预发 setpoint（100次 ≈ 5秒） ---
    ROS_INFO("Pre-streaming setpoints...");
    for (int i = 0; i < 100 && ros::ok(); i++)
    {
        pose.header.stamp = ros::Time::now();
        setpoint_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // --- 步骤6：OFFBOARD + 解锁 + 起飞 ---
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    ROS_INFO("Requesting OFFBOARD and arming...");
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                ROS_INFO("OFFBOARD mode enabled.");
            last_request = ros::Time::now();
        }
        else if (!current_state.armed &&
                 current_state.mode == "OFFBOARD" &&
                 (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                ROS_INFO("Vehicle armed.");
            last_request = ros::Time::now();
        }

        // 起飞完成判定
        if (current_state.armed && current_state.mode == "OFFBOARD" &&
            std::fabs(local_pos.pose.pose.position.z - abs_goal_z) < reach_threshold)
        {
            ROS_INFO("Takeoff complete at height %.2f", local_pos.pose.pose.position.z);
            break;
        }

        pose.header.stamp = ros::Time::now();
        setpoint_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // --- 步骤7：VFH+ 避障飞向目标点 ---
    ROS_INFO("Navigating to goal (%.1f, %.1f) with VFH+ avoidance...", abs_goal_x, abs_goal_y);
    double prev_selected_angle = std::atan2(abs_goal_y - init_y, abs_goal_x - init_x);
    double smoothed_angle = prev_selected_angle;
    ros::Time stuck_start = ros::Time(0);  // 阻塞/紧急悬停开始时间
    bool retreating = false;
    double retreat_x = 0, retreat_y = 0;
    // 记录来时位置，用于后退脱困方向
    double prev_pos_x = local_pos.pose.pose.position.x;
    double prev_pos_y = local_pos.pose.pose.position.y;
    ros::Time prev_pos_time = ros::Time::now();
    double ground_z = init_z;  // 地面高度 = 起飞点高度

    while (ros::ok())
    {
        double dist_to_goal = distance_2d(abs_goal_x, abs_goal_y);
        if (dist_to_goal < reach_threshold)
        {
            ROS_INFO("Goal reached! Distance: %.2f", dist_to_goal);
            break;
        }

        double cur_x = local_pos.pose.pose.position.x;
        double cur_y = local_pos.pose.pose.position.y;
        double cur_z = local_pos.pose.pose.position.z;

        // 每 2 秒记录一次位置，用于计算来时方向
        if ((ros::Time::now() - prev_pos_time).toSec() > 2.0)
        {
            double moved = std::sqrt((cur_x - prev_pos_x)*(cur_x - prev_pos_x) +
                                     (cur_y - prev_pos_y)*(cur_y - prev_pos_y));
            if (moved > 0.3)  // 只在有明显移动时更新
            {
                prev_pos_x = cur_x;
                prev_pos_y = cur_y;
            }
            prev_pos_time = ros::Time::now();
        }

        // 检查点云时效性
        bool cloud_valid = cloud_received && latest_cloud && !latest_cloud->empty()
            && (ros::Time::now() - cloud_stamp).toSec() < cloud_timeout;

        if (cloud_received && !cloud_valid)
            ROS_WARN_THROTTLE(2.0, "Point cloud stale (%.1fs)! Flying blind.",
                (ros::Time::now() - cloud_stamp).toSec());

        // 后退脱困模式：也用 VFH+ 避障
        if (retreating)
        {
            double retreat_dist = distance_2d(retreat_x, retreat_y);
            if (retreat_dist < reach_threshold)
            {
                ROS_INFO("Retreat complete, resuming navigation.");
                retreating = false;
                stuck_start = ros::Time(0);
            }
            else
            {
                double retreat_angle = std::atan2(retreat_y - cur_y, retreat_x - cur_x);
                double r_selected = retreat_angle;
                double r_min_dist = std::numeric_limits<double>::max();

                // PLACEHOLDER_RETREAT_VFH
                // 后退时也用 VFH+ 避障
                if (cloud_valid)
                {
                    double vfh_r = vfh_select_direction(
                        latest_cloud, cur_z, slice_half_height, max_range,
                        num_sectors, obstacle_threshold, enlarge_sectors,
                        multi_layer, layer_offset, ground_z, ground_clearance,
                        retreat_angle, yaw, prev_selected_angle,
                        w_target, w_smooth, w_prev, r_min_dist);
                    if (!std::isnan(vfh_r))
                        r_selected = vfh_r;
                }

                double r_step = std::min(step_distance * 0.5, retreat_dist);
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = cur_x + r_step * std::cos(r_selected);
                pose.pose.position.y = cur_y + r_step * std::sin(r_selected);
                pose.pose.position.z = abs_goal_z;
                pose.pose.orientation = yaw_to_quat(r_selected);
                setpoint_pub.publish(pose);
                prev_selected_angle = r_selected;
                ros::spinOnce();
                rate.sleep();
                continue;
            }
        }

        double goal_angle = std::atan2(abs_goal_y - cur_y, abs_goal_x - cur_x);

        // 有有效点云时执行 VFH+，否则直飞
        double selected_angle = goal_angle;
        double min_obs_dist = std::numeric_limits<double>::max();

        if (cloud_valid)
        {
            double vfh_angle = vfh_select_direction(
                latest_cloud, cur_z, slice_half_height, max_range,
                num_sectors, obstacle_threshold, enlarge_sectors,
                multi_layer, layer_offset, ground_z, ground_clearance,
                goal_angle, yaw, prev_selected_angle,
                w_target, w_smooth, w_prev, min_obs_dist);

            if (std::isnan(vfh_angle))
            {
                // 全部阻塞 → 悬停，超时则后退脱困
                if (stuck_start.isZero())
                    stuck_start = ros::Time::now();

                double stuck_dur = (ros::Time::now() - stuck_start).toSec();

                if (stuck_dur > stuck_timeout)
                {
                    // 沿来时方向的反方向后退（不是目标反方向）
                    double back_angle = std::atan2(prev_pos_y - cur_y, prev_pos_x - cur_x);
                    retreat_x = cur_x + retreat_distance * std::cos(back_angle);
                    retreat_y = cur_y + retreat_distance * std::sin(back_angle);
                    retreating = true;
                    ROS_WARN("Stuck %.1fs, retreating %.1fm toward prev pos!", stuck_dur, retreat_distance);
                    continue;
                }

                ROS_WARN_THROTTLE(1.0, "All sectors blocked (%.1fs)! Hovering...", stuck_dur);
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = cur_x;
                pose.pose.position.y = cur_y;
                pose.pose.position.z = abs_goal_z;
                pose.pose.orientation = yaw_to_quat(yaw);
                setpoint_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            // 紧急悬停：最近障碍物 < safety_radius，也计入 stuck 计时
            if (min_obs_dist < safety_radius)
            {
                if (stuck_start.isZero())
                    stuck_start = ros::Time::now();

                double stuck_dur = (ros::Time::now() - stuck_start).toSec();

                if (stuck_dur > stuck_timeout)
                {
                    double back_angle = std::atan2(prev_pos_y - cur_y, prev_pos_x - cur_x);
                    retreat_x = cur_x + retreat_distance * std::cos(back_angle);
                    retreat_y = cur_y + retreat_distance * std::sin(back_angle);
                    retreating = true;
                    ROS_WARN("Emergency hover %.1fs, retreating!", stuck_dur);
                    continue;
                }

                ROS_WARN_THROTTLE(1.0, "Obstacle too close (%.2fm, %.1fs)! Hovering.",
                    min_obs_dist, stuck_dur);
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = cur_x;
                pose.pose.position.y = cur_y;
                pose.pose.position.z = abs_goal_z;
                pose.pose.orientation = yaw_to_quat(yaw);
                setpoint_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            selected_angle = vfh_angle;
            stuck_start = ros::Time(0);  // 正常飞行，重置阻塞计时
        }

        // 方向低通滤波（EMA）
        smoothed_angle = prev_selected_angle +
            direction_alpha * normalize_angle(selected_angle - prev_selected_angle);
        smoothed_angle = normalize_angle(smoothed_angle);

        // 距离自适应速度控制
        double speed_scale = 1.0;
        if (min_obs_dist < slowdown_radius)
        {
            speed_scale = (min_obs_dist - safety_radius) / (slowdown_radius - safety_radius);
            speed_scale = std::max(0.2, std::min(1.0, speed_scale));
        }
        double step = std::min(step_distance * speed_scale, dist_to_goal);

        // 沿滤波后方向生成步进 setpoint，航向朝向飞行方向
        pose.header.stamp = ros::Time::now();
        pose.pose.position.x = cur_x + step * std::cos(smoothed_angle);
        pose.pose.position.y = cur_y + step * std::sin(smoothed_angle);
        pose.pose.position.z = abs_goal_z;
        pose.pose.orientation = yaw_to_quat(smoothed_angle);
        setpoint_pub.publish(pose);

        prev_selected_angle = smoothed_angle;

        ROS_INFO_THROTTLE(2.0, "Nav: dist=%.1f dir=%.0f° obs=%.1f spd=%.0f%%",
            dist_to_goal, smoothed_angle * 180.0 / M_PI, min_obs_dist, speed_scale * 100);

        ros::spinOnce();
        rate.sleep();
    }

    // --- 步骤8：到达目标，悬停稳定 ---
    ROS_INFO("Hovering at goal for %.1f seconds...", hover_time);
    pose.pose.position.x = abs_goal_x;
    pose.pose.position.y = abs_goal_y;
    pose.pose.position.z = abs_goal_z;
    ros::Time hover_start = ros::Time::now();
    while (ros::ok() && (ros::Time::now() - hover_start < ros::Duration(hover_time)))
    {
        pose.header.stamp = ros::Time::now();
        setpoint_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // --- 步骤9：AUTO.LAND 降落 ---
    mavros_msgs::SetMode land_mode;
    land_mode.request.custom_mode = "AUTO.LAND";
    last_request = ros::Time::now();

    ROS_INFO("Switching to AUTO.LAND...");
    while (ros::ok())
    {
        if (current_state.mode != "AUTO.LAND" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if (set_mode_client.call(land_mode) && land_mode.response.mode_sent)
                ROS_INFO("AUTO.LAND mode enabled.");
            last_request = ros::Time::now();
        }

        if (!current_state.armed)
        {
            ROS_INFO("Landed and disarmed. Mission complete.");
            break;
        }

        pose.header.stamp = ros::Time::now();
        setpoint_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
