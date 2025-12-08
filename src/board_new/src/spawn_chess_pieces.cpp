#include <ros/ros.h>
#include <ros/package.h>
#include <gazebo_msgs/SpawnModel.h>
#include <geometry_msgs/Pose.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <regex>

// 定义棋子结构体
struct ChessPiece {
    std::string name;
    std::string type; // 基础类型 (pion, benteng, etc.)
    double x, y, z;   // 在 board_new.urdf 中的原始坐标
};

// 读取文件内容到字符串
std::string readFile(const std::string& package_name, const std::string& file_name) {
    std::string path = ros::package::getPath(package_name) + "/urdf/" + file_name;
    std::ifstream file(path);
    if (!file.is_open()) {
        ROS_ERROR_STREAM("Could not open file: " << path);
        return "";
    }
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
}

// 根据名字判断应该使用哪个 URDF 文件名 (不带 .urdf 后缀)
std::string getPieceUrdfName(const std::string& name, const std::string& type) {
    // 提取名字中的数字
    std::regex num_regex("(\\d+)");
    std::smatch match;
    int number = 0;
    if (std::regex_search(name, match, num_regex)) {
        number = std::stoi(match[1]);
    }

    bool is_black = false;

    // 逻辑判断 (根据你的要求：pion1-8深色，9-16浅色；其他1-2深色，3-4浅色)
    if (type == "pion") {
        if (number >= 1 && number <= 8) is_black = true;
    } else if (type == "benteng" || type == "jaran" || type == "patih") {
        if (number >= 1 && number <= 2) is_black = true;
    } else if (type == "ster" || type == "raja") {
        if (number == 1) is_black = true;
    }
    
    // 返回对应的文件名
    if (is_black) {
        return type + "_black.urdf";
    } else {
        return type + "_white.urdf";
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "spawn_chess_pieces");
    ros::NodeHandle nh;
    
    // 获取参数
    double board_x = 0.3;
    double board_y = 0.0;
    double board_z = 0.0;
    double board_yaw = 1.5708; // 90度

    // 等待 Gazebo 生成服务
    ros::ServiceClient spawn_client = nh.serviceClient<gazebo_msgs::SpawnModel>("/gazebo/spawn_urdf_model");
    ROS_INFO("Waiting for /gazebo/spawn_urdf_model service...");
    spawn_client.waitForExistence();
    ROS_INFO("Service available.");

    // 1. 生成棋盘
    gazebo_msgs::SpawnModel srv;
    srv.request.model_name = "chessboard";
    srv.request.model_xml = readFile("board_new", "chessboard.urdf");
    srv.request.initial_pose.position.x = board_x;
    srv.request.initial_pose.position.y = board_y;
    srv.request.initial_pose.position.z = board_z;
    
    tf2::Quaternion q_board;
    q_board.setRPY(0, 0, board_yaw); 
    srv.request.initial_pose.orientation.x = q_board.x();
    srv.request.initial_pose.orientation.y = q_board.y();
    srv.request.initial_pose.orientation.z = q_board.z();
    srv.request.initial_pose.orientation.w = q_board.w();

    if (spawn_client.call(srv)) {
        ROS_INFO("Spawned chessboard successfully.");
    } else {
        ROS_ERROR("Failed to spawn chessboard.");
    }

    // 2. 定义所有棋子及其原始坐标
    std::vector<ChessPiece> pieces = {
        // 兵 (Pion) - 16个
        {"pion1", "pion", 0.15164, 0.002, 0.10706},
        {"pion2", "pion", 0.10844, 0.002, 0.10706},
        {"pion3", "pion", 0.06524, 0.002, 0.10706},
        {"pion4", "pion", 0.02204, 0.002, 0.10706},
        {"pion5", "pion", -0.02116, 0.002, 0.10706},
        {"pion6", "pion", -0.06436, 0.002, 0.10706},
        {"pion7", "pion", -0.10756, 0.002, 0.10706},
        {"pion8", "pion", -0.15076, 0.002, 0.10706},
        {"pion9", "pion", -0.15076, 0.002, -0.10706},
        {"pion10", "pion", -0.10756, 0.002, -0.10706},
        {"pion11", "pion", -0.06436, 0.002, -0.10706},
        {"pion12", "pion", -0.02116, 0.002, -0.10706},
        {"pion13", "pion", 0.02204, 0.002, -0.10706},
        {"pion14", "pion", 0.06524, 0.002, -0.10706},
        {"pion15", "pion", 0.10844, 0.002, -0.10706},
        {"pion16", "pion", 0.15164, 0.002, -0.10706},

        // 车 (Benteng) - 4个
        {"benteng1", "benteng", 0.15164, 0.002, 0.15150},
        {"benteng2", "benteng", -0.15239, 0.002, 0.15150},
        {"benteng3", "benteng", -0.15239, 0.002, -0.15150},
        {"benteng4", "benteng", 0.15164, 0.002, -0.15150},

        // 马 (Jaran) - 4个
        {"jaran1", "jaran", 0.10681, 0.002, 0.15150},
        {"jaran2", "jaran", -0.10756, 0.002, 0.15150},
        {"jaran3", "jaran", -0.10756, 0.002, -0.15150},
        {"jaran4", "jaran", 0.10681, 0.002, -0.15150},

        // 象 (Patih) - 4个
        {"patih1", "patih", 0.06396, 0.002, 0.15150},
        {"patih2", "patih", -0.06471, 0.002, 0.15150},
        {"patih3", "patih", -0.06471, 0.002, -0.15150},
        {"patih4", "patih", 0.06396, 0.002, -0.15150},

        // 后 (Ster) - 2个
        {"ster1", "ster", -0.02180, 0.002, 0.15150},
        {"ster2", "ster", -0.02180, 0.002, -0.15150},

        // 王 (Raja) - 2个
        {"raja1", "raja", 0.02108, 0.002, 0.15150},
        {"raja2", "raja", 0.02108, 0.002, -0.15150}
    };

    // 3. 循环生成棋子
    tf2::Quaternion q_piece;
    tf2::Quaternion q_urdf_tilt;
    q_urdf_tilt.setRPY(1.5708, 0, 0);
    
    tf2::Quaternion q_final = q_board * q_urdf_tilt;
    q_final.normalize();

    for (const auto& piece : pieces) {
        gazebo_msgs::SpawnModel piece_srv;
        piece_srv.request.model_name = piece.name;
        
        // 决定使用哪个 URDF 文件
        std::string urdf_file_name = getPieceUrdfName(piece.name, piece.type);
        
        // 读取 URDF
        std::string urdf_xml = readFile("board_new", urdf_file_name);
        
        if (urdf_xml.empty()) {
            ROS_ERROR_STREAM("Skipping " << piece.name << " because URDF file " << urdf_file_name << " not found.");
            continue;
        }

        piece_srv.request.model_xml = urdf_xml;
        
        // 计算位置
        double local_x = piece.x;
        double local_y = -piece.z;
        double local_z = piece.y + 0.02;
        
        double world_x = board_x + (local_x * cos(board_yaw) - local_y * sin(board_yaw));
        double world_y = board_y + (local_x * sin(board_yaw) + local_y * cos(board_yaw));
        double world_z = board_z + local_z;

        piece_srv.request.initial_pose.position.x = world_x;
        piece_srv.request.initial_pose.position.y = world_y;
        piece_srv.request.initial_pose.position.z = world_z;

        piece_srv.request.initial_pose.orientation.x = q_final.x();
        piece_srv.request.initial_pose.orientation.y = q_final.y();
        piece_srv.request.initial_pose.orientation.z = q_final.z();
        piece_srv.request.initial_pose.orientation.w = q_final.w();

        if (spawn_client.call(piece_srv)) {
            ROS_INFO_STREAM("Spawned " << piece.name << " using " << urdf_file_name);
        } else {
            ROS_ERROR_STREAM("Failed to spawn " << piece.name);
        }
        
        ros::Duration(0.05).sleep();
    }
    return 0;
}