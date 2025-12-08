#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <string>
#include <vector>
#include <algorithm>

class PiecesManager {
public:
    PiecesManager() {
        // 订阅 Gazebo 模型状态
        model_states_sub_ = nh_.subscribe("/gazebo/model_states", 10, &PiecesManager::modelStatesCallback, this);
        
        // 定义我们要跟踪的棋子名称前缀
        piece_prefixes_ = {"pion", "benteng", "jaran", "patih", "ster", "raja"};
        
        // 初始化上次发布时间
        last_publish_time_ = ros::Time(0);
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        // 频率控制：限制为 30Hz (0.033s)
        if ((ros::Time::now() - last_publish_time_).toSec() < 0.033) {
            return;
        }
        last_publish_time_ = ros::Time::now();

        // 遍历所有模型
        for (size_t i = 0; i < msg->name.size(); ++i) {
            std::string name = msg->name[i];
            
            // 检查是否是棋子
            if (isChessPiece(name)) {
                // 发布 TF
                geometry_msgs::TransformStamped transformStamped;
                
                transformStamped.header.stamp = ros::Time::now();
                transformStamped.header.frame_id = "world"; // 父坐标系
                transformStamped.child_frame_id = name;     // 子坐标系 (棋子名字)
                
                transformStamped.transform.translation.x = msg->pose[i].position.x;
                transformStamped.transform.translation.y = msg->pose[i].position.y;
                transformStamped.transform.translation.z = msg->pose[i].position.z;
                
                transformStamped.transform.rotation = msg->pose[i].orientation;
                
                tf_broadcaster_.sendTransform(transformStamped);
            }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber model_states_sub_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
    std::vector<std::string> piece_prefixes_;

    // 上次发布时间
    ros::Time last_publish_time_;

    bool isChessPiece(const std::string& name) {
        for (const auto& prefix : piece_prefixes_) {
            // 检查名字是否以这些前缀开头
            // 注意：要避免匹配到类似 "robot_arm" 这种不相关的
            // 我们的棋子名字是 "pion1", "benteng2" 等，没有下划线分隔数字
            if (name.find(prefix) == 0) {
                // 简单的检查：前缀匹配
                // 可以加更严格的检查，比如后面必须跟数字
                return true; 
            }
        }
        return false;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pieces_manager");
    PiecesManager manager;
    ros::spin();
    return 0;
}