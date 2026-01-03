#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <sstream>

// 辅助函数：角度转弧度
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// 辅助函数：鲁棒的运动规划与执行 (关节空间规划)
bool robust_move(moveit::planning_interface::MoveGroupInterface& move_group, geometry_msgs::Pose& target_pose) {
    move_group.setPoseTarget(target_pose);
    
    ROS_INFO_STREAM("planning to " << target_pose.position.x << "," <<
              target_pose.position.y << "," <<
              target_pose.position.z);
              
    std::vector<std::string> planner_ids = { "RRTConnect","RRTstar", "PRM", "TRRT"};
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    for (const auto& planner_id : planner_ids) {
        move_group.setPlannerId(planner_id);
        move_group.setPlanningTime(7.0); 
        
        if (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
            move_group.execute(my_plan);
            return true;
        }
    }
    ROS_ERROR("All planners failed for target pose.");
    return false;
}

// 新增辅助函数：笛卡尔空间直线运动
bool cartesian_move(moveit::planning_interface::MoveGroupInterface& move_group, const geometry_msgs::Pose& target_pose) {
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0; // 禁用跳跃检测
    const double eef_step = 0.01;      // 1cm 插值步长

    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    ROS_INFO_STREAM("Cartesian path fraction: " << fraction * 100.0 << "%");

    if (fraction >= 0.95) { // 允许微小误差
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        my_plan.trajectory_ = trajectory;
        move_group.execute(my_plan);
        return true;
    } else {
        ROS_WARN("Cartesian path incomplete (fraction < 0.95). Aborting.");
        return false;
    }
}

void speedUp(moveit::planning_interface::MoveGroupInterface& action_group){
    action_group.setMaxVelocityScalingFactor(1.0); 
    action_group.setMaxAccelerationScalingFactor(1.0);
}

void speedDown(moveit::planning_interface::MoveGroupInterface& action_group){

    action_group.setMaxVelocityScalingFactor(0.001); 
    action_group.setMaxAccelerationScalingFactor(0.001);
}
// -----------------

// --- 核心函数：执行抓取和放置序列 ---
void execute_pick_and_place(moveit::planning_interface::MoveGroupInterface& arm_group, 
                            moveit::planning_interface::MoveGroupInterface& gripper_group, 
                            geometry_msgs::Pose& pick_pose, 
                            geometry_msgs::Pose& place_pose) 
{
    ROS_INFO(">>> Starting Pick and Place Sequence <<<");



    tf2::Quaternion q;
    q.setRPY(0, 0, 0);   
    pick_pose.orientation = tf2::toMsg(q);
    place_pose.orientation = tf2::toMsg(q);


    // 1. 移动到 up 位姿 (关节空间)
    ROS_INFO("1. Moving to 'up'");
    speedUp(arm_group);
    arm_group.setNamedTarget("up");
    arm_group.move();

    // 2. 移动到 pick_pose 上方 0.15m
    ROS_INFO("2. Moving to Pre-Pick (0.15m above)");
    speedUp(arm_group);
    geometry_msgs::Pose pre_pick = pick_pose;
    pre_pick.position.z += 0.15;

    if (!robust_move(arm_group, pre_pick)) return;

    // 3. 打开夹爪至 33 度
    ROS_INFO("3. Opening gripper (33 deg)");
    gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint", deg2rad(33.0));
    gripper_group.move();

    // 4. 移动到 pick_pose (笛卡尔空间 - 垂直下落)
    ROS_INFO("4. Moving to Pick Pose [Cartesian]");
    speedDown(arm_group);
    if (!cartesian_move(arm_group, pick_pose)) {
        ROS_ERROR("Failed to approach pick pose.");
        return;
    }


    // 5. 合并夹爪至 37 度 (抓取)
    ROS_INFO("5. Closing gripper (37 deg)");
    gripper_group.setMaxVelocityScalingFactor(0.05);
    gripper_group.setMaxAccelerationScalingFactor(0.1);

    gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint", deg2rad(37.0));
    gripper_group.move();

    // 6. 移动到 pick_pose 上方 0.15m (笛卡尔空间 - 垂直抬起)
    ROS_INFO("6. Lifting up (0.15m above) [Cartesian]");
    speedDown(arm_group);
    if (!cartesian_move(arm_group, pre_pick)) return;

    // 7. 移动到 place_pose 上方 0.15m (笛卡尔空间 - 平移)
    ROS_INFO("7. Moving to Pre-Place (0.15m above) [Cartesian]");
    speedUp(arm_group);
    geometry_msgs::Pose pre_place = place_pose;
    pre_place.position.z += 0.15;
    if (!cartesian_move(arm_group, pre_place)) {
         ROS_WARN("Cartesian move to Pre-Place failed, trying joint space...");
         if (!robust_move(arm_group, pre_place)) return;
    }

    // 8. 移动到 place_pose (笛卡尔空间 - 垂直下落)
    ROS_INFO("8. Moving to Place Pose [Cartesian]");
    speedDown(arm_group);
    if (!cartesian_move(arm_group, place_pose)) {
        ROS_ERROR("Failed to approach place pose.");
        arm_group.setMaxVelocityScalingFactor(0.8);
        return;
    }
    arm_group.setMaxVelocityScalingFactor(0.8);

    // 9. 打开夹爪至 0.412 rad (释放)
    ROS_INFO("9. Opening gripper ( 0.412 rad)");
    gripper_group.setMaxVelocityScalingFactor(0.05);
    gripper_group.setMaxAccelerationScalingFactor(0.1);
    gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.412);
    gripper_group.move();

    // 10. 移动到 place_pose 上方 0.15m (撤离)
    // 这里通常也建议用笛卡尔，保持垂直撤离
    ROS_INFO("10. Lifting up from Place [Cartesian]");
    speedDown(arm_group);
    if (!cartesian_move(arm_group, pre_place)) {
        robust_move(arm_group, pre_place);
    }

    // 11. 回到 up 位姿
    ROS_INFO("11. Returning to 'up'");
    speedUp(arm_group);
    arm_group.setNamedTarget("up");
    arm_group.move();

    ROS_INFO(">>> Pick and Place Complete <<<");
}

class ChessMoveNode {
public:
    ChessMoveNode(ros::NodeHandle& nh) 
        : arm_group("my_arm"), gripper_group("my_gripper"), tf_listener(tf_buffer) 
    {
        // 初始设置
        arm_group.setMaxVelocityScalingFactor(0.1); 
        arm_group.setMaxAccelerationScalingFactor(0.1);
        arm_group.setPoseReferenceFrame("world");
        arm_group.allowReplanning(true);
        arm_group.setNumPlanningAttempts(100);
        
        // 夹爪设置
        gripper_group.setMaxVelocityScalingFactor(0.1);
        gripper_group.setMaxAccelerationScalingFactor(0.1);

        // 容差设置
        arm_group.setGoalPositionTolerance(0.01); 
        arm_group.setGoalOrientationTolerance(0.05); 
        arm_group.setPlanningTime(10.0);

        // 订阅话题
        sub = nh.subscribe("/chess_move", 10, &ChessMoveNode::moveCallback, this);
        
        ROS_INFO("ChessMoveNode initialized. Waiting for commands on /chess_move...");
        
        // 等待 TF 缓存
        ros::Duration(1.0).sleep();
    }

    void moveCallback(const std_msgs::String::ConstPtr& msg) {
        std::stringstream ss(msg->data);
        std::string piece_name;
        double x, y, z;
        ss >> piece_name >> x >> y >> z;

        if (ss.fail()) {
            ROS_ERROR("Failed to parse command: %s", msg->data.c_str());
            return;
        }

        ROS_INFO("Received Move Command: Piece='%s', Target=(%.4f, %.4f, %.4f)", piece_name.c_str(), x, y, z);

        geometry_msgs::Pose pick_pose;
        bool found = false;
        try {
            geometry_msgs::TransformStamped transformStamped;
            transformStamped = tf_buffer.lookupTransform("world", piece_name, ros::Time(0), ros::Duration(1.0));

            pick_pose.position.x = transformStamped.transform.translation.x;
            pick_pose.position.y = transformStamped.transform.translation.y;
            pick_pose.position.z = transformStamped.transform.translation.z;
            pick_pose.orientation = transformStamped.transform.rotation;
            found = true;
        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could not find transform for '%s': %s", piece_name.c_str(), ex.what());
            return;
        }

        if (found) {
            geometry_msgs::Pose place_pose;
            place_pose.position.x = x;
            place_pose.position.y = y;
            place_pose.position.z = z;
            place_pose.orientation = pick_pose.orientation; // 保持抓取时的姿态

            // 执行抓取放置
            execute_pick_and_place(arm_group, gripper_group, pick_pose, place_pose);
        }
    }

private:
    moveit::planning_interface::MoveGroupInterface arm_group;
    moveit::planning_interface::MoveGroupInterface gripper_group;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;
    ros::Subscriber sub;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "move_given_pieces");
    ros::NodeHandle nh;
    
    ros::AsyncSpinner spinner(2);
    spinner.start();

    ChessMoveNode chess_move_node(nh);

    ros::waitForShutdown();
    return 0;
}