#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <robot_arm_control/GetJointTrajectory.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

enum State {
    WAITING_FOR_TARGET,
    PLANNING_PATH,
    EXECUTING_PATH
};

class FSMNode {
public:
    FSMNode() : tf_listener_(tf_buffer_), current_state_(WAITING_FOR_TARGET) {
        // 订阅目标指令
        target_sub_ = nh_.subscribe("/chess_bot/target_piece", 10, &FSMNode::targetCallback, this);
        
        // 规划服务客户端
        plan_client_ = nh_.serviceClient<robot_arm_control::GetJointTrajectory>("plan_path");

        // 初始化关节发布者
        for (int i = 0; i < 5; ++i) {
            std::string topic = "/arm/robot_arm/joint" + std::to_string(i + 1) + "_position_controller/command";
            joint_pubs_.push_back(nh_.advertise<std_msgs::Float64>(topic, 10));
        }

        ROS_INFO("FSM Node Initialized. State: WAITING_FOR_TARGET");
    }

    void run() {
        ros::Rate rate(10); // 10Hz
        while (ros::ok()) {
            switch (current_state_) {
                case WAITING_FOR_TARGET:
                    // 只是等待回调函数接收目标
                    break;

                case PLANNING_PATH:
                    executePlanning();
                    break;

                case EXECUTING_PATH:
                    monitorExecution();
                    break;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber target_sub_;
    ros::ServiceClient plan_client_;
    std::vector<ros::Publisher> joint_pubs_;
    
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    State current_state_;
    std::string target_piece_name_;
    trajectory_msgs::JointTrajectory current_trajectory_;
    ros::Time execution_start_time_;
    ros::Time planning_start_time_; // 新增：规划开始时间

    void targetCallback(const std_msgs::String::ConstPtr& msg) {
        if (current_state_ == WAITING_FOR_TARGET) {
            target_piece_name_ = msg->data;
            ROS_INFO("Received target: %s. Transitioning to PLANNING.", target_piece_name_.c_str());
            current_state_ = PLANNING_PATH;
            planning_start_time_ = ros::Time::now(); // 记录开始时间
        }
    }

    void executePlanning() {
        // 检查是否超时 (3秒)
        if (ros::Time::now() - planning_start_time_ > ros::Duration(3.0)) {
            ROS_WARN("Planning timed out for target: %s. Returning to WAITING.", target_piece_name_.c_str());
            current_state_ = WAITING_FOR_TARGET;
            return;
        }

        robot_arm_control::GetJointTrajectory srv;
        
        try {
            // 1. 查询 TF 获取棋子位置
            // 修改：去掉 "board/" 前缀，直接使用 target_piece_name_ (例如 "pion1")
            // 因为 pieces_manager 发布的是 "world" -> "pion1"
            geometry_msgs::TransformStamped transform;
            // 注意：这里如果查不到 TF 会抛出异常，也会导致重试，直到超时
            transform = tf_buffer_.lookupTransform("arm/base_link", target_piece_name_, ros::Time(0), ros::Duration(1.0));
            
            srv.request.target_pose.x = transform.transform.translation.x;
            srv.request.target_pose.y = transform.transform.translation.y;
            
            // 修改：增加抓取高度偏移 (例如 8cm)
            // 这样夹爪中心会在棋子上方，而不是试图砸进地里
            srv.request.target_pose.z = transform.transform.translation.z + 0.08; 

            // 2. 调用 Motion Planner
            if (plan_client_.call(srv)) {
                if (srv.response.success) {
                    current_trajectory_ = srv.response.trajectory;
                    ROS_INFO("Path planned successfully. Transitioning to EXECUTING.");
                    
                    // 开始执行
                    publishTrajectoryPoint(current_trajectory_.points[0]);
                    execution_start_time_ = ros::Time::now();
                    current_state_ = EXECUTING_PATH;
                } else {
                    ROS_WARN_THROTTLE(1.0, "Motion Planner failed to find a path. Retrying...");
                    // 保持在 PLANNING 状态重试，直到超时
                }
            } else {
                ROS_ERROR("Failed to call service plan_path");
                // 服务调用失败通常是严重的，直接回退
                current_state_ = WAITING_FOR_TARGET;
            }

        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(1.0, "Could not find transform for %s: %s", target_piece_name_.c_str(), ex.what());
            // 找不到 TF，保持在 PLANNING 状态重试，直到超时
        }
    }

    void monitorExecution() {
        // 简单的监控逻辑：检查时间是否超时，或者检查末端误差
        // 这里我们简单实现：假设2秒后应该到达
        
        // 持续发布目标位置（防止丢包）
        if (!current_trajectory_.points.empty()) {
             publishTrajectoryPoint(current_trajectory_.points[0]);
        }

        // 检查是否超时 (假设给3秒时间执行)
        if (ros::Time::now() - execution_start_time_ > ros::Duration(3.0)) {
            // 这里应该加入实际位置检测逻辑 (通过 /joint_states 或 TF)
            // 如果误差过大 -> current_state_ = PLANNING_PATH (Replan)
            
            ROS_INFO("Execution finished (assumed). Waiting for next target.");
            current_state_ = WAITING_FOR_TARGET;
        }
    }

    void publishTrajectoryPoint(const trajectory_msgs::JointTrajectoryPoint& point) {
        for (size_t i = 0; i < point.positions.size() && i < joint_pubs_.size(); ++i) {
            std_msgs::Float64 msg;
            msg.data = point.positions[i];
            joint_pubs_[i].publish(msg);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fsm_node");
    FSMNode fsm;
    fsm.run();
    return 0;
}