#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char* argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
      "hello_moveit", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto arm_group = MoveGroupInterface(node, "robot_arm");
  auto gripper_group = MoveGroupInterface(node, "robot_gripper");

  RCLCPP_INFO(logger, "Step 1: Moving Arm to Target...");

  // Set a target Pose
  auto const target_pose = [] {
    geometry_msgs::msg::Pose msg;
    msg.orientation.w = 1.0;
    msg.position.x = 0.28;
    msg.position.y = -0.2;
    msg.position.z = 0.5;
    return msg;
  }();
  arm_group.setPoseTarget(target_pose);
  
  // 规划并执行 (使用 move() 简化版，它会自动规划+执行)
  auto result = arm_group.move();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Arm movement failed!");
    return 1;
  }

  // ==========================================
  // 步骤 2: 打开夹爪
  // ==========================================
  RCLCPP_INFO(logger, "Step 2: Opening Gripper...");
  
  // 使用你在 Setup Assistant 里定义的 "open" 姿态
  gripper_group.setNamedTarget("open"); 
  
  result = gripper_group.move();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Gripper open failed!");
    return 1;
  }

  // (可选) 等待一小会儿，让动作看起来更自然
  rclcpp::sleep_for(std::chrono::seconds(1));

  // ==========================================
  // 步骤 3: 闭合夹爪
  // ==========================================
  RCLCPP_INFO(logger, "Step 3: Closing Gripper...");
  
  // 使用你在 Setup Assistant 里定义的 "close" 姿态
  gripper_group.setNamedTarget("close");
  
  result = gripper_group.move();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Gripper close failed!");
    return 1;
  }

  RCLCPP_INFO(logger, "All tasks completed successfully!");
  rclcpp::shutdown();
  return 0;
}

