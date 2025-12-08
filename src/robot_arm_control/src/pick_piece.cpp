#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <string>
#include <cmath>

// 定义机械臂关节数量
const int JOINT_COUNT = 5; 

class PickAndPlaceNode {
public:
    PickAndPlaceNode() : tf_listener_(tf_buffer_) {
        // 初始化发布者，用于控制各个关节
        joint_pubs_.resize(JOINT_COUNT);
        for (int i = 0; i < JOINT_COUNT; ++i) {
            // 修改话题前缀：从 /robot_arm 改为 /arm/robot_arm
            std::string topic = "/arm/robot_arm/joint" + std::to_string(i + 1) + "_position_controller/command";
            joint_pubs_[i] = nh_.advertise<std_msgs::Float64>(topic, 10);
        }

        // 初始化夹爪发布者
        // 修改话题前缀：从 /robot_arm 改为 /arm/robot_arm
        gripper_pub_ = nh_.advertise<std_msgs::Float64>("/arm/robot_arm/joint6left_position_controller/command", 10);

        ROS_INFO("PickAndPlace Node Initialized. Waiting for TF...");
    }

    // 核心功能：抓取指定棋子
    void pickPiece(const std::string& piece_name) {
        geometry_msgs::TransformStamped transform;
        std::string target_frame = "board/" + piece_name; // 棋子的 TF 帧名
        std::string base_frame = "arm/base_link";         // 机械臂基座 TF 帧名

        try {
            // 1. 查询 TF 树，获取棋子相对于机械臂基座的位置
            transform = tf_buffer_.lookupTransform(base_frame, target_frame, ros::Time(0), ros::Duration(3.0));
            
            double x = transform.transform.translation.x;
            double y = transform.transform.translation.y;
            double z = transform.transform.translation.z;

            ROS_INFO("Found piece '%s' at relative pos: [x: %.3f, y: %.3f, z: %.3f]", 
                     piece_name.c_str(), x, y, z);

            // 2. 移动到棋子上方 (预备位置，高出 5cm)
            // 注意：这里的 z 是棋子中心，可能需要根据棋子高度微调
            moveArmTo(x, y, z + 0.15); 
            ros::Duration(3.0).sleep();

            // 3. 打开夹爪
            controlGripper(true); 
            ros::Duration(1.0).sleep();

            // 4. 下降到抓取位置 (稍微高一点点，避免撞倒)
            moveArmTo(x, y, z + 0.08);
            ros::Duration(3.0).sleep();

            // 5. 闭合夹爪 (抓取)
            controlGripper(false);
            ros::Duration(1.0).sleep();

            // 6. 抬起
            moveArmTo(x, y, z + 0.20);
            ros::Duration(2.0).sleep();

            ROS_INFO("Pick sequence completed.");

        } catch (tf2::TransformException &ex) {
            ROS_WARN("Could not transform '%s' to '%s': %s", target_frame.c_str(), base_frame.c_str(), ex.what());
        }
    }

private:
    ros::NodeHandle nh_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<ros::Publisher> joint_pubs_;
    ros::Publisher gripper_pub_;

    // 控制夹爪: open=true 张开, open=false 闭合
    void controlGripper(bool open) {
        std_msgs::Float64 msg;
        // 根据 URDF limit: lower="-0.2" upper="1.0"
        // 0.0 是闭合状态，0.5 是张开状态
        msg.data = open ? 0.5 : 0.0; 
        gripper_pub_.publish(msg);
        ROS_INFO("Gripper: %s", open ? "OPEN" : "CLOSE");
    }

    // 移动机械臂末端到指定 (x, y, z)
    void moveArmTo(double x, double y, double z) {
        std::vector<double> joint_angles;
        
        if (calculateIK(x, y, z, joint_angles)) {
            // 发布关节角度
            for (size_t i = 0; i < joint_angles.size() && i < joint_pubs_.size(); ++i) {
                std_msgs::Float64 msg;
                msg.data = joint_angles[i];
                joint_pubs_[i].publish(msg);
            }
            ROS_INFO("Moving to [%.2f, %.2f, %.2f]", x, y, z);
        } else {
            ROS_ERROR("IK Solution not found for [%.2f, %.2f, %.2f]", x, y, z);
        }
    }

    // 几何逆运动学求解器
    bool calculateIK(double target_x, double target_y, double target_z, std::vector<double>& angles) {
        // 机械臂连杆长度 (单位: 米)
        // L1: Base to Shoulder (Joint 2)
        const double L1 = 0.09; 
        // L2: Upper Arm (Joint 2 to Joint 3)
        const double L2 = 0.12; 
        // L3: Forearm (Joint 3 to Joint 4)
        const double L3 = 0.0916; 
        // L4: Wrist to End Effector (Joint 4 to Grasping Frame)
        const double L4 = 0.138; 

        angles.resize(5);

        // ---------------------------------------------------------
        // 1. Joint 1 (Base Yaw): 控制水平方向
        // ---------------------------------------------------------
        angles[0] = atan2(target_y, target_x);

        // ---------------------------------------------------------
        // 2. 计算平面内的目标点 (r, z)
        // 我们将 3D 问题简化为 2D 平面 (由 Joint 1 旋转确定的平面)
        // ---------------------------------------------------------
        double r_target = sqrt(target_x * target_x + target_y * target_y);
        double z_target = target_z - L1; // 相对于 Shoulder 的高度

        // ---------------------------------------------------------
        // 3. 确定手腕位置 (Wrist Position)
        // 我们假设末端执行器垂直向下抓取 (Pitch = -90度)
        // 这样 Joint 2, 3, 4 的总角度应为 -PI/2
        // ---------------------------------------------------------
        double pitch_goal = -M_PI / 2.0; // 垂直向下

        // 手腕中心 (Joint 4) 的坐标
        double r_wrist = r_target - L4 * cos(pitch_goal); // cos(-90) = 0, 所以 r_wrist = r_target
        double z_wrist = z_target - L4 * sin(pitch_goal); // sin(-90) = -1, 所以 z_wrist = z_target + L4

        // ---------------------------------------------------------
        // 4. 求解 Joint 2 和 Joint 3 (两连杆 IK)
        // ---------------------------------------------------------
        // 手腕到 Shoulder 的距离
        double D = sqrt(r_wrist * r_wrist + z_wrist * z_wrist);

        // 检查是否超出工作空间
        if (D > (L2 + L3) || D < fabs(L2 - L3)) {
            ROS_ERROR("Target out of reach! Distance: %.3f, Max Reach: %.3f", D, L2 + L3);
            return false;
        }

        // 使用余弦定理求解 Joint 3 (Elbow)
        // c3 = (r^2 + z^2 - L2^2 - L3^2) / (2 * L2 * L3)
        double c3 = (r_wrist * r_wrist + z_wrist * z_wrist - L2 * L2 - L3 * L3) / (2 * L2 * L3);
        // 限制 c3 范围以防数值误差
        if (c3 > 1.0) c3 = 1.0;
        if (c3 < -1.0) c3 = -1.0;
        
        // s3 有正负两个解 (Elbow Up / Elbow Down)，通常选正值 (Elbow Up)
        double s3 = sqrt(1 - c3 * c3);
        
        // Joint 3 角度 (注意：URDF 中 Joint 3 的 origin rpy="-3.1416 0 0"，可能需要调整符号)
        // 在标准几何中，这是肘部弯曲角度
        double theta3 = atan2(s3, c3);

        // 求解 Joint 2 (Shoulder)
        // theta2 = atan2(z, r) - atan2(L3*s3, L2 + L3*c3)
        double theta2 = atan2(z_wrist, r_wrist) - atan2(L3 * s3, L2 + L3 * c3);

        // ---------------------------------------------------------
        // 5. 求解 Joint 4 (Wrist Pitch)
        // ---------------------------------------------------------
        // 全局 Pitch = theta2 + theta3 + theta4
        // 我们希望全局 Pitch = -PI/2 (垂直向下)
        // 所以 theta4 = -PI/2 - theta2 - theta3
        double theta4 = pitch_goal - theta2 - theta3;

        // ---------------------------------------------------------
        // 6. 赋值并处理 URDF 偏移
        // ---------------------------------------------------------
        
        // Joint 2: URDF 中有 rpy="1.5708 0 1.5708"，这通常意味着坐标系旋转了
        // 这里的几何解是基于标准 DH 参数的，可能需要根据实际 URDF 调整偏移
        // 假设 Joint 2 0度是垂直向上，我们需要减去 PI/2 变成水平
        angles[1] = -theta2 + M_PI/2; // 尝试修正方向

        // Joint 3: URDF origin rpy="-3.1416 0 0"
        angles[2] = -theta3; 

        // Joint 4: URDF origin rpy="1.5708 0 0"
        angles[3] = -theta4;

        // Joint 5 (Wrist Roll): 保持 0 度即可，或者根据需要旋转
        angles[4] = 0.0;

        // 简单的关节限位检查 (根据 URDF limit)
        // Joint 2: -1.57 ~ 1.57
        // Joint 3: -2.09 ~ 2.09
        // Joint 4: -2.09 ~ 2.09
        
        // 打印调试信息
        ROS_INFO("IK Result: J1=%.2f, J2=%.2f, J3=%.2f, J4=%.2f", angles[0], angles[1], angles[2], angles[3]);

        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pick_piece_node");
    
    PickAndPlaceNode node;

    // 等待系统稳定
    ros::Duration(1.0).sleep();

    // 假设我们要抓取 "pion1" (兵1)
    std::string piece_to_pick = "pion1";
    if (argc > 1) {
        piece_to_pick = argv[1];
    }

    node.pickPiece(piece_to_pick);

    ros::spin();
    return 0;
}