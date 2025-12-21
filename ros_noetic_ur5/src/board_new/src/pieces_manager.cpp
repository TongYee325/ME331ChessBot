#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <vector>
#include <algorithm>
#include <set>

// MoveIt & Geometric Shapes
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>
#include <shape_msgs/Mesh.h>

class PiecesManager {
public:
    PiecesManager() {
        // 订阅 Gazebo 模型状态
        model_states_sub_ = nh_.subscribe("/gazebo/model_states", 10, &PiecesManager::modelStatesCallback, this);
        
        // 发布过滤后的棋子状态话题
        pieces_states_pub_ = nh_.advertise<gazebo_msgs::ModelStates>("/chess_pieces_states", 10);

        // 定义我们要跟踪的棋子名称前缀
        piece_prefixes_ = {"pion", "benteng", "jaran", "patih", "ster", "raja"};
        
        // 等待 MoveIt 连接建立
        ros::Duration(1.0).sleep();
        ROS_INFO("PiecesManager initialized and waiting for Gazebo states...");
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        // 简单频率控制：每收到 10 条消息处理一次 (Gazebo 默认 1000Hz，这里降频到 100Hz)
        static int counter = 0;
        if (counter++ < 10) return;
        counter = 0;

        gazebo_msgs::ModelStates pieces_msg;
        std::vector<moveit_msgs::CollisionObject> collision_objects;
        bool found_any = false;

        // 遍历所有模型
        for (size_t i = 0; i < msg->name.size(); ++i) {
            std::string name = msg->name[i];
            
            // 检查是否是棋子
            if (isChessPiece(name)) {
                found_any = true;

                // 1. 收集数据用于发布话题
                pieces_msg.name.push_back(name);
                pieces_msg.pose.push_back(msg->pose[i]);
                pieces_msg.twist.push_back(msg->twist[i]);

                // 2. 发布 TF
                geometry_msgs::TransformStamped transformStamped;
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "world";
                transformStamped.child_frame_id = name;
                transformStamped.transform.translation.x = msg->pose[i].position.x;
                transformStamped.transform.translation.y = msg->pose[i].position.y;
                transformStamped.transform.translation.z = msg->pose[i].position.z;
                transformStamped.transform.rotation = msg->pose[i].orientation;
                tf_broadcaster_.sendTransform(transformStamped);
            }
        }

        // 发布话题
        if (!pieces_msg.name.empty()) {
            pieces_states_pub_.publish(pieces_msg);
            // 每 5 秒打印一次日志证明活着
            ROS_INFO_THROTTLE(5.0, "Publishing /chess_pieces_states with %lu pieces", pieces_msg.name.size());
        } else {
            ROS_WARN_THROTTLE(5.0, "No chess pieces found in Gazebo states!");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber model_states_sub_;
    ros::Publisher pieces_states_pub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::vector<std::string> piece_prefixes_;

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

    bool isChessPiece(const std::string& name) {
        for (const auto& prefix : piece_prefixes_) {
            if (name.find(prefix) == 0) return true; 
        }
        return false;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pieces_manager");
    
    // 使用 AsyncSpinner 确保 MoveIt 的回调能正常处理
    ros::AsyncSpinner spinner(2);
    spinner.start();

    PiecesManager manager;
    
    ros::waitForShutdown();
    return 0;
}