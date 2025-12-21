#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <gazebo_msgs/ModelStates.h>
#include <tf2/LinearMath/Quaternion.h>
#include <map>
#include <string>
#include <vector>

// 全局变量存储棋子位置
std::map<std::string, geometry_msgs::Pose> g_pieces_poses;
bool g_pieces_received = false;

void piecesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    for (size_t i = 0; i < msg->name.size(); ++i) {
        g_pieces_poses[msg->name[i]] = msg->pose[i];
    }
    if (!msg->name.empty()) {
        g_pieces_received = true;
    }
}

void operateGripper(moveit::planning_interface::MoveGroupInterface& gripper_group, bool open) {
    std::string target_name = open ? "open" : "close";
    gripper_group.setNamedTarget(target_name);
    gripper_group.move();
    ROS_INFO("Gripper %s", open ? "OPENED" : "CLOSED");
}

// [新增] 辅助函数：执行笛卡尔直线运动
bool moveCartesian(moveit::planning_interface::MoveGroupInterface& group, const geometry_msgs::Pose& target_pose) {
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(target_pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    
    double fraction = group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    
    ROS_INFO("Cartesian path fraction: %.2f%%", fraction * 100.0);

    if (fraction >= 0.9) { // 如果规划成功率超过 90%
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        return (group.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    } else {
        ROS_WARN("Cartesian path planning failed (fraction too low).");
        return false;
    }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "motion_planner_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  ros::Subscriber sub = nh.subscribe("/chess_pieces_states", 10, piecesCallback);

  static const std::string ARM_GROUP = "my_arm";
  static const std::string GRIPPER_GROUP = "my_gripper";

  moveit::planning_interface::MoveGroupInterface arm_group(ARM_GROUP);
  arm_group.setEndEffectorLink("wrist_3_link");
  
  // [修改] 设置规划器 ID (尝试 RRTConnect，通常比较快且稳定)
  arm_group.setPlannerId("RRTConnect");
  
  arm_group.setPlanningTime(10.0);
  arm_group.setNumPlanningAttempts(10); // 增加尝试次数
  arm_group.setGoalPositionTolerance(0.01);
  arm_group.setGoalOrientationTolerance(0.05);
  arm_group.setMaxVelocityScalingFactor(0.1);
  arm_group.setMaxAccelerationScalingFactor(0.1);

  moveit::planning_interface::MoveGroupInterface gripper_group(GRIPPER_GROUP);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  arm_group.setPoseReferenceFrame("world");

  ROS_INFO("Waiting for chess pieces info...");
  ros::Rate rate(10);
  while (ros::ok() && !g_pieces_received) {
      rate.sleep();
  }
  ROS_INFO("Received chess pieces info.");

  geometry_msgs::Pose target_pose;
  std::string target_piece_name = "pion1";
  const double GRIPPER_OFFSET = 0.16; // 稍微增加一点补偿，防止撞击

  if (g_pieces_poses.count(target_piece_name)) {
      geometry_msgs::Pose piece_pose = g_pieces_poses[target_piece_name];
      ROS_INFO("Target %s found at [%.3f, %.3f, %.3f]", 
               target_piece_name.c_str(), piece_pose.position.x, piece_pose.position.y, piece_pose.position.z);

      tf2::Quaternion q;
      q.setRPY(M_PI, 0, 0); 
      target_pose.orientation.x = q.x();
      target_pose.orientation.y = q.y();
      target_pose.orientation.z = q.z();
      target_pose.orientation.w = q.w();

      // 1. 移动到预备位置 (使用普通规划)
      target_pose.position = piece_pose.position;
      target_pose.position.z += 0.20 + GRIPPER_OFFSET; 
      
      ROS_INFO("Moving to pre-grasp position...");
      arm_group.setPoseTarget(target_pose);
      if (arm_group.move() != moveit::planning_interface::MoveItErrorCode::SUCCESS) {
          ROS_ERROR("Failed to move to pre-grasp position");
          return 1;
      }

      // 2. 打开夹爪
      operateGripper(gripper_group, true);

      // 3. 下降抓取 (使用笛卡尔直线规划)
      target_pose.position.z = piece_pose.position.z + GRIPPER_OFFSET; 
      ROS_INFO("Descending to grasp...");
      if (!moveCartesian(arm_group, target_pose)) {
          ROS_ERROR("Failed to descend!");
          // 如果笛卡尔失败，尝试普通规划作为备选
          arm_group.setPoseTarget(target_pose);
          arm_group.move();
      }

      // 4. 闭合夹爪
      operateGripper(gripper_group, false);

      // 5. 附加物体
      moveit_msgs::AttachedCollisionObject attached_object;
      attached_object.link_name = arm_group.getEndEffectorLink();
      attached_object.object.header.frame_id = "world";
      attached_object.object.id = target_piece_name;
      attached_object.object.operation = attached_object.object.ADD;
      planning_scene_interface.applyAttachedCollisionObject(attached_object);
      ROS_INFO("Attached object %s", target_piece_name.c_str());

      // 6. 抬起 (使用笛卡尔直线规划)
      target_pose.position.z += 0.20;
      ROS_INFO("Lifting object...");
      moveCartesian(arm_group, target_pose);

      // 7. 移动到目标放置位置 (普通规划)
      target_pose.position.x += 0.1;
      target_pose.position.y += 0.1;
      ROS_INFO("Moving to drop position...");
      arm_group.setPoseTarget(target_pose);
      arm_group.move();

      // 8. 放下
      operateGripper(gripper_group, true);

      // 9. 解除附加
      attached_object.object.operation = attached_object.object.REMOVE;
      planning_scene_interface.applyAttachedCollisionObject(attached_object);
      ROS_INFO("Detached object");

      // 10. 抬起离开 (笛卡尔)
      target_pose.position.z += 0.10;
      moveCartesian(arm_group, target_pose);

      ROS_INFO("Mission Complete!");

  } else {
      ROS_ERROR("Target piece %s not found!", target_piece_name.c_str());
  }

  ros::shutdown();
  return 0;
}