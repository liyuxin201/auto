#include "ros/ros.h"
#include "mavros_msgs/PositionTarget.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <array>
#include <thread>
#include "cmath"
#include "algorithm"
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>

#define PI 3.14159265358979323846

#define STRAIGHT_CTRL      1
#define EGO_CTRL           2
#define GATE_CTRL          3

uint8_t ctrl_mode_flag = 2;

float safe_area_xmax;
float safe_area_ymax;
float safe_area_hightmax;
float vx_max, vy_max, vz_max;
double tracking_kp_x, tracking_kp_y, tracking_kp_z;
double control_cmd_timeout;

static ros::Publisher local_pose_vel_pub;
static ros::ServiceClient set_mode_client;
static ros::Subscriber state_sub;
static ros::Subscriber local_position_pose_sub;
static ros::Subscriber control_data_sub;
static ros::Subscriber straight_control_data_sub;
static ros::Subscriber ego_control_data_sub;
static ros::Subscriber flag_sub;

volatile bool received_local_pose = false;

static ros::Subscriber rcin_sub;
static ros::Subscriber rc_override_sub;
static ros::ServiceClient arming_client;

static mavros_msgs::SetMode offb_set_mode;
static mavros_msgs::CommandBool arm_cmd;
mavros_msgs::PositionTarget control_data;

static uint8_t fightarea_check_flag = 1;
static uint8_t fightarea_flag_change_allow = 1;
static uint8_t mode_check_flag = 1;
static uint8_t mode_select_flag = 0;
static uint8_t arm_select_flag = 0;
static uint16_t rc_data_channel6_last = 0;
static ros::Time guided_enter_time;
static ros::Time last_control_cmd_time;
static bool guided_hold_active = false;
static bool received_control_cmd = false;
static bool control_cmd_has_position_reference = false;

ros::Timer timer_safe_check;
ros::Timer timer_mode_change;

static mavros_msgs::State current_state;
static mavros_msgs::RCIn raw_rc_data;
static mavros_msgs::RCIn rc_data;
static std::array<uint16_t, 18> rc_override_channels;
static geometry_msgs::PoseStamped local_pose_data;
static mavros_msgs::PositionTarget local_pose_data_last;
static mavros_msgs::PositionTarget local_pose_data_boforeOutarea;

std::mutex control_data_mutex;
volatile bool send_setpoint_enable = true;

double getYawFromPose(const geometry_msgs::PoseStamped& pose_msg)
{
    geometry_msgs::Quaternion quat = pose_msg.pose.orientation;
    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(quat,tf_quat);
    double roll,pitch,yaw;
    tf::Matrix3x3(tf_quat).getRPY(roll,pitch,yaw);
    return yaw;
}

void apply_rc_override_to_effective_input()
{
    rc_data = raw_rc_data;
    if (rc_data.channels.size() < rc_override_channels.size())
        rc_data.channels.resize(rc_override_channels.size(), 0);
    for (size_t i = 0; i < rc_override_channels.size(); ++i)
    {
        const uint16_t override_value = rc_override_channels[i];
        if (override_value != mavros_msgs::OverrideRCIn::CHAN_RELEASE &&
            override_value != mavros_msgs::OverrideRCIn::CHAN_NOCHANGE)
        {
            rc_data.channels[i] = override_value;
        }
    }
}

void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void rc_input_callback(const mavros_msgs::RCIn::ConstPtr& msg)
{
    raw_rc_data = *msg;
    apply_rc_override_to_effective_input();
}

void rc_override_callback(const mavros_msgs::OverrideRCIn::ConstPtr& msg)
{
    const size_t count = std::min(rc_override_channels.size(), msg->channels.size());
    for (size_t i = 0; i < count; ++i)
    {
        if (msg->channels[i] == mavros_msgs::OverrideRCIn::CHAN_NOCHANGE)
            continue;
        rc_override_channels[i] = msg->channels[i];
    }
    apply_rc_override_to_effective_input();
}

void localpositionposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    received_local_pose = true;
    local_pose_data = *msg;
}

void mark_control_cmd_received(bool has_position_reference)
{
    last_control_cmd_time = ros::Time::now();
    received_control_cmd = true;
    control_cmd_has_position_reference = has_position_reference;
}

// ===================== 【速度控制函数】 =====================
int command_velocity_control(double vx, double vy, double vz, float yaw)
{
    if (mode_check_flag != 0 || fightarea_check_flag != 0)
    {
        if (fightarea_check_flag != 0)
            local_pose_vel_pub.publish(local_pose_data_boforeOutarea);
        return 0;
    }

    mavros_msgs::PositionTarget vel_cmd;
    vel_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;

    // 速度模式：忽略所有位置，只使用速度 + yaw
    vel_cmd.type_mask =
        mavros_msgs::PositionTarget::IGNORE_PX |
        mavros_msgs::PositionTarget::IGNORE_PY |
        mavros_msgs::PositionTarget::IGNORE_PZ |
        mavros_msgs::PositionTarget::IGNORE_AFX |
        mavros_msgs::PositionTarget::IGNORE_AFY |
        mavros_msgs::PositionTarget::IGNORE_AFZ |
        mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    vel_cmd.velocity.x = vx;
    vel_cmd.velocity.y = vy;
    vel_cmd.velocity.z = vz;
    vel_cmd.yaw = yaw;

    local_pose_vel_pub.publish(vel_cmd);
    return 1;
}

void safe_check_Callback(const ros::TimerEvent &event)
{
    switch (mode_select_flag)
    {
        case 1:
        case 2:
            if (current_state.mode != "GUIDED")
            {
                mode_check_flag = 1;
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                    ROS_INFO("guided enable");
            }
            else
            {
                mode_check_flag = 0;
                if (arm_select_flag == 1 && !current_state.armed)
                {
                    if (arming_client.call(arm_cmd) && arm_cmd.response.success)
                        ROS_INFO("Vehicle armed");
                }
            }
            ROS_INFO(mode_select_flag == 1 ? "command position" : "command control");
            break;
    }

    if (fabs(local_pose_data.pose.position.x) > safe_area_xmax ||
        fabs(local_pose_data.pose.position.y) > safe_area_ymax ||
        fabs(local_pose_data.pose.position.z) > safe_area_hightmax)
    {
        if (fightarea_flag_change_allow)
        {
            fightarea_check_flag = 1;
            fightarea_flag_change_allow = 0;
            send_setpoint_enable = false;
            ROS_ERROR("fightarea_check_flag change to 0");
        }
        ROS_ERROR("The UAV is out of the safe area");
    }
    else
    {
        if (fightarea_flag_change_allow)
        {
            local_pose_data_boforeOutarea.position.x = local_pose_data.pose.position.x;
            local_pose_data_boforeOutarea.position.y = local_pose_data.pose.position.y;
            local_pose_data_boforeOutarea.position.z = local_pose_data.pose.position.z;
            local_pose_data_boforeOutarea.yaw = getYawFromPose(local_pose_data);
            fightarea_check_flag = 0;
            ROS_ERROR("The UAV is in the safe area");
        }
    }
}

void mode_check_Callback(const ros::TimerEvent &event)
{
    static uint16_t rc_data_channel5_last = 0;

    if (!((rc_data.channels[4] > rc_data_channel5_last - 20) &&
          (rc_data.channels[4] < rc_data_channel5_last + 20)))
    {
        ROS_INFO("mode change detected (ch5 changed from %d to %d)",
                 rc_data_channel5_last, rc_data.channels[4]);

        local_pose_data_last.position.x = local_pose_data.pose.position.x;
        local_pose_data_last.position.y = local_pose_data.pose.position.y;
        local_pose_data_last.position.z = local_pose_data.pose.position.z;
        local_pose_data_last.yaw = getYawFromPose(local_pose_data);

        control_data.velocity.x = 0.0;
        control_data.velocity.y = 0.0;
        control_data.velocity.z = 0.0;
        control_data.yaw = getYawFromPose(local_pose_data);

        if (rc_data.channels[4] >= 1850)
        {
            mode_select_flag = 1;
            offb_set_mode.request.custom_mode = "GUIDED";
            send_setpoint_enable = true;
            guided_enter_time = ros::Time::now();
            guided_hold_active = true;
            ROS_INFO("Mode: GUIDED HOLD (case 1)");
        }
        else
        {
            mode_select_flag = 0;
            offb_set_mode.request.custom_mode = "LOITER";
            send_setpoint_enable = false;
            guided_hold_active = false;
            if (set_mode_client.call(offb_set_mode)) {
                ROS_INFO("Switch to LOITER success!");
            } else {
                ROS_WARN("Switch to LOITER failed");
            }
            ROS_INFO("Mode: LOITER");
        }
        fightarea_flag_change_allow = 1;
        fightarea_check_flag = 0;
    }

    if (mode_select_flag == 1 && guided_hold_active)
    {
        if ((ros::Time::now() - guided_enter_time).toSec() >= 3.0)
        {
            mode_select_flag = 2;
            guided_hold_active = false;
            ROS_INFO("=== HOLD -> COMMAND FOLLOWING ===");
        }
    }

    rc_data_channel5_last = rc_data.channels[4];
    arm_select_flag = (rc_data.channels[5] >= 1850) ? 0 : 1;

    ROS_INFO_THROTTLE(1, "arm:%d mode:%d send:%d ch5=%d ch6=%d",
                      arm_select_flag, mode_select_flag, send_setpoint_enable,
                      rc_data.channels[4], rc_data.channels[5]);
}

void straightControldataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("Get control straight ctrl data");
    if (ctrl_mode_flag == STRAIGHT_CTRL)
    {
        std::lock_guard<std::mutex> lock(control_data_mutex);
        control_data.velocity.x = msg->pose.position.x;
        control_data.velocity.y = msg->pose.position.y;
        control_data.velocity.z = msg->pose.position.z;
        control_data.yaw = getYawFromPose(local_pose_data);
        mark_control_cmd_received(false);
        ROS_INFO("Straight ctrl data accept");
    }
}

// ===================== 【接收规划轨迹并转换为速度跟踪目标】 =====================
void EGOControldataCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    if (ctrl_mode_flag == EGO_CTRL)
    {
        if (!msg->header.frame_id.empty() && msg->header.frame_id != "world")
        {
            ROS_WARN_THROTTLE(1.0, "Ignore /position_cmd with frame_id=%s", msg->header.frame_id.c_str());
            return;
        }

        ROS_INFO("Get EGO ctrl data");
        std::lock_guard<std::mutex> lock(control_data_mutex);
        control_data.position.x = msg->position.x;
        control_data.position.y = msg->position.y;
        control_data.position.z = msg->position.z;
        control_data.velocity.x = msg->velocity.x;
        control_data.velocity.y = msg->velocity.y;
        control_data.velocity.z = msg->velocity.z;
        control_data.yaw = msg->yaw;
        mark_control_cmd_received(true);
        ROS_INFO("ego ctrl data accept");
    }
}

void FlagCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    ctrl_mode_flag = msg->data;
}

void velocity_and_position_limit(mavros_msgs::PositionTarget& msg)
{
    msg.velocity.x = std::max(std::min(msg.velocity.x, static_cast<double>(vx_max)), -static_cast<double>(vx_max));
    msg.velocity.y = std::max(std::min(msg.velocity.y, static_cast<double>(vy_max)), -static_cast<double>(vy_max));
    msg.velocity.z = std::max(std::min(msg.velocity.z, static_cast<double>(vz_max)), -static_cast<double>(vz_max));
}

int px4control_init(ros::NodeHandle nh)
{
    offb_set_mode.request.custom_mode = "GUIDED";
    arm_cmd.request.value = true;

    local_pose_data_last.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    local_pose_data_last.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_VX |
                                     mavros_msgs::PositionTarget::IGNORE_VY |
                                     mavros_msgs::PositionTarget::IGNORE_VZ;
    local_pose_data_last.position.x = 0.0;
    local_pose_data_last.position.y = 0.0;
    local_pose_data_last.position.z = 0.0;
    local_pose_data_last.yaw = 0.0;

    local_pose_data_boforeOutarea.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    local_pose_data_boforeOutarea.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                              mavros_msgs::PositionTarget::IGNORE_AFY |
                                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                                              mavros_msgs::PositionTarget::IGNORE_VX |
                                              mavros_msgs::PositionTarget::IGNORE_VY |
                                              mavros_msgs::PositionTarget::IGNORE_VZ;
    local_pose_data_boforeOutarea.position.x = 0.0;
    local_pose_data_boforeOutarea.position.y = 0.0;
    local_pose_data_boforeOutarea.position.z = 0.0;
    local_pose_data_boforeOutarea.yaw = 0.0;

    timer_safe_check = nh.createTimer(ros::Duration(0.2), safe_check_Callback);
    timer_mode_change = nh.createTimer(ros::Duration(0.1), mode_check_Callback);
    timer_safe_check.stop();
    timer_mode_change.stop();

    fightarea_check_flag = 1;
    fightarea_flag_change_allow = 1;
    mode_check_flag = 1;
    mode_select_flag = 0;
    last_control_cmd_time = ros::Time(0);
    raw_rc_data.channels.assign(18, 0);
    rc_data.channels.assign(18, 0);
    rc_override_channels.fill(mavros_msgs::OverrideRCIn::CHAN_RELEASE);

    ros::param::get("flight_area/flight_area_xmax", safe_area_xmax);
    ros::param::get("flight_area/flight_area_ymax", safe_area_ymax);
    ros::param::get("flight_area/flight_area_hightmax", safe_area_hightmax);
    ros::param::get("flight_velocity/vx", vx_max);
    ros::param::get("flight_velocity/vy", vy_max);
    ros::param::get("flight_velocity/vz", vz_max);
    nh.param("velocity_tracking/kp_x", tracking_kp_x, 0.8);
    nh.param("velocity_tracking/kp_y", tracking_kp_y, 0.8);
    nh.param("velocity_tracking/kp_z", tracking_kp_z, 1.0);
    nh.param("velocity_tracking/cmd_timeout", control_cmd_timeout, 0.2);
    ROS_INFO("safe_area_xmax:%f", safe_area_xmax);
    ROS_INFO("safe_area_ymax:%f", safe_area_ymax);
    ROS_INFO("safe_area_hightmax:%f", safe_area_hightmax);
    ROS_INFO("vx_max:%f", vx_max);
    ROS_INFO("vy_max:%f", vy_max);
    ROS_INFO("vz_max:%f", vz_max);
    ROS_INFO("tracking_kp_x:%f", tracking_kp_x);
    ROS_INFO("tracking_kp_y:%f", tracking_kp_y);
    ROS_INFO("tracking_kp_z:%f", tracking_kp_z);
    ROS_INFO("control_cmd_timeout:%f", control_cmd_timeout);

    state_sub = nh.subscribe("mavros/state", 10, state_callback);
    rcin_sub = nh.subscribe("mavros/rc/in", 10, rc_input_callback);
    rc_override_sub = nh.subscribe("mavros/rc/override", 10, rc_override_callback);
    local_position_pose_sub = nh.subscribe("mavros/local_position/pose", 10, localpositionposeCallback);
    straight_control_data_sub = nh.subscribe<geometry_msgs::PoseStamped>("straight_output_target", 2, straightControldataCallback);
    ego_control_data_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd", 2, EGOControldataCallback);
    flag_sub = nh.subscribe("/control_data/task_status", 1, FlagCallback);

    ROS_INFO("等待接收初始位姿...");
    ros::Rate rate(5);
    while (ros::ok() && !received_local_pose)
    {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "初始位姿：" << local_pose_data.pose.position.x << " "
              << local_pose_data.pose.position.y << " "
              << local_pose_data.pose.position.z << std::endl;

    control_data.position.x = local_pose_data.pose.position.x;
    control_data.position.y = local_pose_data.pose.position.y;
    control_data.position.z = local_pose_data.pose.position.z;
    control_data.velocity.x = 0.0;
    control_data.velocity.y = 0.0;
    control_data.velocity.z = 0.0;
    control_data.yaw = getYawFromPose(local_pose_data);
    ROS_INFO("已接收初始位姿，继续后续初始化!");

    local_pose_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    return 1;
}

int px4control_start()
{
    ros::Rate rate(20);
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("connected with px4");
    timer_mode_change.start();
    ros::Duration du(0.5);
    du.sleep();
    ROS_INFO("mode change check start");
    timer_safe_check.start();
    du.sleep();
    return 1;
}

// ===================== 【主循环：速度前馈 + 位置误差反馈】 =====================
void uavcontrol()
{
    ros::Rate rate(30);
    while (ros::ok())
    {
        if (!send_setpoint_enable)
        {
            rate.sleep();
            continue;
        }

        switch (mode_select_flag)
        {
            case 1:
                local_pose_vel_pub.publish(local_pose_data_last);
                break;
            case 2:
            {
                mavros_msgs::PositionTarget tracked_cmd;
                bool cmd_stale = false;
                bool use_position_feedback = false;

                {
                    std::lock_guard<std::mutex> lock(control_data_mutex);
                    tracked_cmd = control_data;
                    cmd_stale = !received_control_cmd ||
                                (ros::Time::now() - last_control_cmd_time).toSec() > control_cmd_timeout;
                    use_position_feedback = control_cmd_has_position_reference && ctrl_mode_flag == EGO_CTRL;
                }

                if (cmd_stale)
                {
                    tracked_cmd.velocity.x = 0.0;
                    tracked_cmd.velocity.y = 0.0;
                    tracked_cmd.velocity.z = 0.0;
                    tracked_cmd.yaw = getYawFromPose(local_pose_data);
                    ROS_WARN_THROTTLE(1.0, "Control command timeout, brake with zero velocity");
                }
                else if (use_position_feedback)
                {
                    tracked_cmd.velocity.x += tracking_kp_x * (tracked_cmd.position.x - local_pose_data.pose.position.x);
                    tracked_cmd.velocity.y += tracking_kp_y * (tracked_cmd.position.y - local_pose_data.pose.position.y);
                    tracked_cmd.velocity.z += tracking_kp_z * (tracked_cmd.position.z - local_pose_data.pose.position.z);
                }

                velocity_and_position_limit(tracked_cmd);
                command_velocity_control(
                    tracked_cmd.velocity.x,
                    tracked_cmd.velocity.y,
                    tracked_cmd.velocity.z,
                    tracked_cmd.yaw);
                ROS_INFO("give command to uav");
                break;
            }
        }
        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "px4_control");
    ros::NodeHandle nh;
    ROS_INFO("OK");
    px4control_init(nh);
    ROS_INFO("contrl init sucess");
    while (!px4control_start());
    ROS_INFO("control start sucess");
    std::thread control_thread(uavcontrol);
    ros::spin();
    return 0;
}
