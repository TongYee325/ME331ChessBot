#include <memory>
#include <cmath> // For atan2, sin, cos

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Helper function to calculate Quaternion for 5-DOF arm
// We assume the gripper needs to point "down" (Pitch = 90 deg)
// And the Yaw must align with the direction to the target (atan2(y, x))
geometry_msgs::msg::Quaternion calculateOrientation(double x, double y) {
    geometry_msgs::msg::Quaternion q;
    double yaw = std::atan2(y, x);
    double pitch = 1.5708; // 90 degrees (Pointing down)
    double roll = 0.0;

    // Euler to Quaternion conversion
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);

    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

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

  // Increase planning time for difficult IK
  arm_group.setPlanningTime(10.0);
  arm_group.setNumPlanningAttempts(20);
  // Relax tolerances slightly to help the solver
  arm_group.setGoalPositionTolerance(0.005);
  arm_group.setGoalOrientationTolerance(0.05);

  // Create Planning Scene Interface
  using moveit::planning_interface::PlanningSceneInterface;
  auto planning_scene_interface = PlanningSceneInterface();

  // ==========================================
  // Add Collision Objects (Chess Board & Piece)
  // ==========================================
  
  // 1. Define the Chess Board (as a Box)
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = arm_group.getPlanningFrame();
  collision_object.id = "chess_board";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.3; // x length
  primitive.dimensions[1] = 0.3; // y length
  primitive.dimensions[2] = 0.02; // z thickness

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.22; // Move further away to avoid base collision
  box_pose.position.y = 0.0;
  box_pose.position.z = 0.01; 

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  // 2. Define a Chess Piece (as a Cylinder)
  moveit_msgs::msg::CollisionObject chess_piece;
  chess_piece.header.frame_id = arm_group.getPlanningFrame();
  chess_piece.id = "chess_pawn";

  shape_msgs::msg::SolidPrimitive cylinder;
  cylinder.type = cylinder.CYLINDER;
  cylinder.dimensions.resize(2);
  cylinder.dimensions[0] = 0.06; // height
  cylinder.dimensions[1] = 0.01; // radius

  geometry_msgs::msg::Pose piece_pose;
  piece_pose.orientation.w = 1.0;
  piece_pose.position.x = 0.15; // Target at 15cm
  piece_pose.position.y = 0.0; 
  piece_pose.position.z = 0.05; 

  chess_piece.primitives.push_back(cylinder);
  chess_piece.primitive_poses.push_back(piece_pose);
  chess_piece.operation = chess_piece.ADD;

  // Apply to scene
  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);
  collision_objects.push_back(chess_piece);
  planning_scene_interface.applyCollisionObjects(collision_objects);

  RCLCPP_INFO(logger, "Added Chess Board and Piece to the scene.");
  rclcpp::sleep_for(std::chrono::seconds(2));

  RCLCPP_INFO(logger, "Step 1: Opening Gripper...");
  gripper_group.setNamedTarget("open");
  gripper_group.move();

  // ==========================================
  // Step 2: Move to Pre-Grasp Pose (Position Only)
  // ==========================================
  RCLCPP_INFO(logger, "Step 2: Moving to Pre-Grasp Pose...");
  
  // Target Position (Above the pawn)
  // We use x=0.15 which is very close and reachable
  double target_x = 0.15;
  double target_y = 0.0; 
  double target_z = 0.15;

  arm_group.setStartStateToCurrentState();
  arm_group.setPositionTarget(target_x, target_y, target_z);
  
  // Relax orientation tolerance
  arm_group.setGoalOrientationTolerance(3.14); // Allow full rotation freedom
  
  auto result = arm_group.move();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Pre-grasp movement failed! Error Code: %d", result.val);
    // Fallback: Try a joint-space target that is known to be "forward and down"
    RCLCPP_INFO(logger, "Attempting fallback joint target...");
    std::vector<double> fallback_joints = {0.0, 0.5, 1.0, 0.0, 0.0};
    arm_group.setJointValueTarget(fallback_joints);
    result = arm_group.move();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
         return 1;
    }
  }

  // ==========================================
  // Step 3: Move to Grasp Pose (Lower down)
  // ==========================================
  RCLCPP_INFO(logger, "Step 3: Approaching Pawn...");
  
  // Try Cartesian Path for straight line approach
  std::vector<geometry_msgs::msg::Pose> waypoints;
  geometry_msgs::msg::Pose target_pose = arm_group.getCurrentPose().pose;
  
  target_pose.position.z -= 0.05; // Go down 5cm
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  
  double fraction = arm_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  
  if (fraction > 0.9) {
      arm_group.execute(trajectory);
  } else {
      RCLCPP_WARN(logger, "Cartesian path failed (%.2f%%), using standard planning...", fraction * 100.0);
      arm_group.setPositionTarget(target_x, target_y, 0.10);
      arm_group.move();
  }

  // ==========================================
  // Step 4: Close Gripper (Seize)
  // ==========================================
  RCLCPP_INFO(logger, "Step 4: Closing Gripper...");
  gripper_group.setNamedTarget("close");
  gripper_group.move();

  // ==========================================
  // Step 5: Attach Object to Gripper
  // ==========================================
  RCLCPP_INFO(logger, "Step 5: Attaching Object...");
  
  moveit_msgs::msg::AttachedCollisionObject attached_object;
  attached_object.link_name = "link5"; // The link the object is attached to
  attached_object.object.header.frame_id = "link5";
  attached_object.object.id = "chess_pawn";
  attached_object.object.operation = attached_object.object.ADD;

  planning_scene_interface.applyAttachedCollisionObject(attached_object);

  // ==========================================
  // Step 6: Lift Object
  // ==========================================
  RCLCPP_INFO(logger, "Step 6: Lifting Object...");
  
  target_z = 0.20; // Lift up
  arm_group.setPositionTarget(target_x, target_y, target_z);
  arm_group.move();

  // ==========================================
  // Step 7: Move to Final Place (New Location)
  // ==========================================
  RCLCPP_INFO(logger, "Step 7: Moving to Final Place...");
  
  // New location: x=0.0, y=0.20 (To the left)
  target_x = 0.0;
  target_y = 0.20;
  target_z = 0.20; // Stay high
  
  arm_group.setPositionTarget(target_x, target_y, target_z);
  result = arm_group.move();
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(logger, "Move to final place failed!");
  } else {
      // Lower to place
      RCLCPP_INFO(logger, "Step 8: Lowering to Place...");
      target_z = 0.10;
      arm_group.setPositionTarget(target_x, target_y, target_z);
      arm_group.move();
      
      // Open Gripper
      RCLCPP_INFO(logger, "Step 9: Releasing Object...");
      gripper_group.setNamedTarget("open");
      gripper_group.move();
      
      // Detach
      moveit_msgs::msg::AttachedCollisionObject detach_object;
      detach_object.object.id = "chess_pawn";
      detach_object.link_name = "link5";
      detach_object.object.operation = detach_object.object.REMOVE;
      planning_scene_interface.applyAttachedCollisionObject(detach_object);
  }

  RCLCPP_INFO(logger, "Mission Complete!");
  rclcpp::shutdown();
  return 0;
}

