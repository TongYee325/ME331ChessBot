#include <ros/ros.h>
#include <robot_arm_control/GetJointTrajectory.h>
#include <geometry_msgs/Point.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <cmath>
#include <vector>

// 机械臂参数 (与 pick_piece.cpp 保持一致)
const double L1 = 0.09; 
const double L2 = 0.12; 
const double L3 = 0.0916; 
const double L4 = 0.138; 

class MotionPlanner {
public:
    MotionPlanner() {
        service_ = nh_.advertiseService("plan_path", &MotionPlanner::planCallback, this);
        ROS_INFO("Motion Planner Ready.");
    }

    bool planCallback(robot_arm_control::GetJointTrajectory::Request &req,
                      robot_arm_control::GetJointTrajectory::Response &res) {
        
        ROS_INFO("Planning request received for [%.3f, %.3f, %.3f]", req.target_pose.x, req.target_pose.y, req.target_pose.z);

        std::vector<double> angles;
        if (calculateIK(req.target_pose.x, req.target_pose.y, req.target_pose.z, angles)) {
            res.success = true;
            
            trajectory_msgs::JointTrajectoryPoint point;
            point.positions = angles;
            point.time_from_start = ros::Duration(2.0); // 假设2秒到达

            res.trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5"};
            res.trajectory.points.push_back(point);
            
            ROS_INFO("Plan success!");
        } else {
            res.success = false;
            ROS_WARN("Plan failed: IK no solution.");
        }
        return true;
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;

    bool calculateIK(double target_x, double target_y, double target_z, std::vector<double>& angles) {
        angles.resize(5);

        // 1. Joint 1 (Base Yaw)
        angles[0] = atan2(target_y, target_x);
        
        // 添加调试打印
        ROS_INFO("IK Debug: Target(%.3f, %.3f, %.3f) -> J1 Angle: %.3f deg", 
                 target_x, target_y, target_z, angles[0] * 180.0 / M_PI);

        // 2. 平面投影
        double r_target = sqrt(target_x * target_x + target_y * target_y);
        double z_target = target_z - L1; // 相对于 Shoulder 的高度

        // 3. 手腕位置 (假设垂直向下抓取 Pitch = -90度)
        double pitch_goal = -M_PI / 2.0;
        double r_wrist = r_target - L4 * cos(pitch_goal);
        double z_wrist = z_target - L4 * sin(pitch_goal);

        // 4. Joint 2 & 3
        double D = sqrt(r_wrist * r_wrist + z_wrist * z_wrist);
        if (D > (L2 + L3) || D < fabs(L2 - L3)) return false;

        double c3 = (r_wrist * r_wrist + z_wrist * z_wrist - L2 * L2 - L3 * L3) / (2 * L2 * L3);
        if (c3 > 1.0) c3 = 1.0;
        if (c3 < -1.0) c3 = -1.0;
        
        double s3 = sqrt(1 - c3 * c3);
        double theta3 = atan2(s3, c3);
        double theta2 = atan2(z_wrist, r_wrist) - atan2(L3 * s3, L2 + L3 * c3);

        // 5. Joint 4
        double theta4 = pitch_goal - theta2 - theta3;

        // 6. URDF 修正
        angles[1] = -theta2 + M_PI/2; 
        angles[2] = -theta3; 
        angles[3] = -theta4;
        angles[4] = 0.0; // Joint 5 Roll

        ROS_INFO("IK Input: x=%.3f, y=%.3f -> J1=%.3f", target_x, target_y, angles[0]);

        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "motion_planner_node");
    MotionPlanner planner;
    ros::spin();
    return 0;
}