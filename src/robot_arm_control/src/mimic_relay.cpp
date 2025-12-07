
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <string>
#include <map>

class MimicRelay
{
public:
    MimicRelay()
    {
        // 获取参数
        ros::NodeHandle nh_private("~");
        
        // 主关节名称
        if (!nh_private.getParam("master_joint", master_joint_name_))
        {
            master_joint_name_ = "joint6_left"; // 默认值
        }

        // 定义从动关节控制器名称和倍率
        // 这里硬编码了你的配置，也可以改为从参数服务器读取
        mimic_map_["joint7left_position_controller"] = 1.0;
        mimic_map_["joint8left_position_controller"] = 1.0;
        mimic_map_["joint9right_position_controller"] = 1.0;
        mimic_map_["joint10right_position_controller"] = 1.0;
        mimic_map_["joint11right_position_controller"] = 1.0;

        // 初始化发布者
        for (auto const& [controller, multiplier] : mimic_map_)
        {
            std::string topic = "/robot_arm/" + controller + "/command";
            pubs_[controller] = nh_.advertise<std_msgs::Float64>(topic, 1);
        }

        // 订阅关节状态
        sub_ = nh_.subscribe("/robot_arm/joint_states", 1, &MimicRelay::jointStateCallback, this);
        
        ROS_INFO("Mimic Relay Node (C++) Started. Master: %s", master_joint_name_.c_str());
    }

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
    {
        // 查找主关节的索引
        auto it = std::find(msg->name.begin(), msg->name.end(), master_joint_name_);
        
        if (it != msg->name.end())
        {
            int index = std::distance(msg->name.begin(), it);
            double current_angle = msg->position[index];

            // 向所有从动关节发送命令
            for (auto const& [controller, multiplier] : mimic_map_)
            {
                std_msgs::Float64 cmd_msg;
                cmd_msg.data = current_angle * multiplier;
                pubs_[controller].publish(cmd_msg);
            }
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    std::map<std::string, ros::Publisher> pubs_;
    std::map<std::string, double> mimic_map_;
    std::string master_joint_name_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mimic_relay_cpp");
    MimicRelay relay;
    ros::spin();
    return 0;
}