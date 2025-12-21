#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <cmath>
// 新增头文件
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

double deg2rad(double degrees) {
    return degrees * M_PI / 180.0;
}

// 辅助函数：移动机械臂到指定关节角度
bool moveArmToJoints(moveit::planning_interface::MoveGroupInterface& group, const std::vector<double>& joints) {
    group.setJointValueTarget(joints);
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        return (group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }
    return false;
}

// 辅助函数：控制夹爪到指定角度
void moveGripper(moveit::planning_interface::MoveGroupInterface& group, double angle_deg) {
    std::vector<double> joints = group.getCurrentJointValues();
    if (joints.empty()) return;

    double target_rad = deg2rad(angle_deg);
    // 简单的限位保护 (根据你的夹爪实际范围调整，通常 Robotiq 85 是 0~0.8)
    if (target_rad > 0.8) target_rad = 0.8;
    if (target_rad < 0.0) target_rad = 0.0;

    joints[0] = target_rad;
    group.setJointValueTarget(joints);
    group.move();
}

// 新增：获取棋子位姿的函数
geometry_msgs::Pose getPiecePose(const std::string& piece_name, const std::string& reference_frame, tf2_ros::Buffer& tf_buffer) {
    geometry_msgs::Pose pose;
    try {
        // 查询 piece_name 相对于 reference_frame 的变换
        // 注意：棋子的 TF 名字通常就是棋子的名字，例如 "pion1", "benteng1"
        // 如果你的 pieces_manager 发布的 TF 有前缀（如 "board/pion1"），请加上前缀
        geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(reference_frame, piece_name, ros::Time(0), ros::Duration(3.0));
        
        pose.position.x = transformStamped.transform.translation.x;
        pose.position.y = transformStamped.transform.translation.y;
        pose.position.z = transformStamped.transform.translation.z;
        pose.orientation = transformStamped.transform.rotation;
        
        ROS_INFO_STREAM("Got pose for " << piece_name << ": [" << pose.position.x << ", " << pose.position.y << ", " << pose.position.z << "]");
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Could not get pose for %s: %s", piece_name.c_str(), ex.what());
    }
    return pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "custom_position_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(2);
  spinner.start();

  // 新增：初始化 TF 监听器
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);

  static const std::string ARM_GROUP = "my_arm";
  static const std::string GRIPPER_GROUP = "my_gripper";

  moveit::planning_interface::MoveGroupInterface arm_group(ARM_GROUP);
  moveit::planning_interface::MoveGroupInterface gripper_group(GRIPPER_GROUP);

  // 1. 设置参考坐标系为 "world"
  arm_group.setPoseReferenceFrame("world");

  // 2. 获取并打印当前受控的末端 Link 名字
  std::string ee_link = arm_group.getEndEffectorLink();
  std::string ref_frame = arm_group.getPoseReferenceFrame();

  ROS_INFO("--------------------------------------------------");
  ROS_INFO("Current Reference Frame: %s", ref_frame.c_str());
  ROS_INFO("Target Link (End Effector): %s", ee_link.c_str());
  ROS_INFO("Any pose target you set will be applied to '%s' relative to '%s'", ee_link.c_str(), ref_frame.c_str());
  ROS_INFO("--------------------------------------------------");

  arm_group.setPlanningTime(10.0);
  arm_group.setNumPlanningAttempts(10);
  arm_group.setMaxVelocityScalingFactor(1.0); 
  arm_group.setMaxAccelerationScalingFactor(1.0);

  // 1. 移动到 "up" 位姿
  ROS_INFO("1. Moving to 'up' pose...");
  arm_group.setNamedTarget("up");
  arm_group.move();

  // 2. 移动到第一组关节位置
  ROS_INFO("2. Moving to Pose 1...");
  std::vector<double> pose1 = {0.235, -2.264, 2.693, -1.994, -1.551, 0.220};
  if (moveArmToJoints(arm_group, pose1)) {
      ROS_INFO("Reached Pose 1.");
  } else {
      ROS_ERROR("Failed to reach Pose 1.");
  }

  // 3. 夹爪打开到 33 度
  ROS_INFO("3. Opening gripper to 33 degrees...");
  moveGripper(gripper_group, 33.0);

  // 4. 移动到第二组关节位置
  ROS_INFO("4. Moving to Pose 2...");
  std::vector<double> pose2 = {0.242, -2.007, 2.836, -2.394, -1.551, 0.227};
  if (moveArmToJoints(arm_group, pose2)) {
      ROS_INFO("Reached Pose 2.");
  } else {
      ROS_ERROR("Failed to reach Pose 2.");
  }




  // 5. 夹爪闭合到 37.5 度
  ROS_INFO("5. Closing gripper to 37.5 degrees...");
  moveGripper(gripper_group, 37.5);

  // 6. 移动到第三组关节位置
  ROS_INFO("6. Moving to Pose 3...");
  std::vector<double> pose3 = {0.235, -2.264, 2.693, -1.994, -1.551, 0.220};
  if (moveArmToJoints(arm_group, pose3)) {
      ROS_INFO("Reached Pose 3.");
  } else {
      ROS_ERROR("Failed to reach Pose 3.");
  }

  // 7. 移动到第4组关节位置
  ROS_INFO("7. Moving to Pose 4...");
  std::vector<double> pose4 = {-0.333,-2.039,2.532,-2.080,-1.546,-0.347};
  if (moveArmToJoints(arm_group, pose4)) {
      ROS_INFO("Reached Pose 4.");
  } else {
      ROS_ERROR("Failed to reach Pose 4.");
  }

  // 8. 移动到第5组关节位置
  ROS_INFO("8. Moving to Pose 5...");
  std::vector<double> pose5 = {-0.349,-1.732,2.695,-2.549,-1.546,-0.363};
  if (moveArmToJoints(arm_group, pose5)) {
      ROS_INFO("Reached Pose 5.");
  } else {
      ROS_ERROR("Failed to reach Pose 5.");
  }

  // 9. 夹爪打开到25度
  ROS_INFO("9. Opening gripper to 25 degrees...");
  moveGripper(gripper_group, 25.0);

  // 10. 移动到 "up" 位姿
  ROS_INFO("10. Moving to 'up' pose...");
  arm_group.setNamedTarget("up");
  arm_group.move();


  // 示例：获取 "pion1" 的位姿
  // 假设我们要抓取 pion1，我们需要知道它相对于 world 的位置 (因为上面设置了参考系为 world)
  ROS_INFO("Waiting for TF data...");
  ros::Duration(1.0).sleep(); // 给 TF 一点时间缓存
  
  std::string target_piece = "pion1"; // 你想抓取的棋子名字
  // 修改：这里查询相对于 "world" 的坐标，以匹配 arm_group 的参考系
  geometry_msgs::Pose piece_pose = getPiecePose(target_piece, "world", tf_buffer);

  // 打印出来验证一下
  if (piece_pose.position.x != 0.0) {
      ROS_INFO_STREAM("Target " << target_piece << " is at: x=" << piece_pose.position.x << ", y=" << piece_pose.position.y);
      
      // 你可以在这里根据 piece_pose 计算抓取点
      // 例如：抓取点在棋子上方 15cm
      // geometry_msgs::Pose pre_grasp_pose = piece_pose;
      // pre_grasp_pose.position.z += 0.15;
      // arm_group.setPoseTarget(pre_grasp_pose);
      // arm_group.move();
  }

  ROS_INFO("Sequence finished.");
  ros::shutdown();
  return 0;
}