#!/usr/bin/env python
# filepath: /home/tongyee/ur_ws/src/board_new/src/simple_manager.py
import rospy
import tf2_ros
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import TransformStamped, Pose
# 新增依赖
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

class ChessPiecesManager:
    def __init__(self):
        rospy.init_node('python_pieces_manager')

        # 1. 话题发布者
        self.pieces_pub = rospy.Publisher('/chess_pieces_states', ModelStates, queue_size=10)
        # 新增：碰撞对象发布者
        self.collision_object_pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=10)

        # 2. TF 广播器
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # 3. 状态记录
        self.prefixes = ["pion", "benteng", "jaran", "patih", "ster", "raja"]
        self.last_stamp = rospy.Time(0) 

        # 4. 添加地面平台 (防止撞地)
        rospy.sleep(1.0) # 等待发布者建立连接
        self.add_ground_platform()

        # 5. 订阅 Gazebo 状态
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.callback)
        
        rospy.loginfo("Python Pieces Manager Started (TF Publisher + Ground Platform)")
        rospy.spin()

    def add_ground_platform(self):
        co = CollisionObject()
        co.header.frame_id = "world"
        co.id = "ground_platform"
        
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [2.0, 2.0, 0.05] # 长2m, 宽2m, 厚5cm

        pose = Pose()
        pose.position.x = 0.0 
        pose.position.y = 0.0
        pose.position.z = -0.025 # 表面在 z=0 (中心在 -0.025)
        pose.orientation.w = 1.0

        co.primitives.append(primitive)
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        self.collision_object_pub.publish(co)
        rospy.loginfo("Added ground platform collision object (2m x 2m)")

    def callback(self, data):
        # 简单的限流，防止 TF 发布频率过高（虽然 Gazebo 默认 1000Hz，但 model_states 也是高频）
        # 这里不做严格限流，尽量实时
        current_time = rospy.Time.now()
        
        # 确保时间戳单调递增
        if current_time <= self.last_stamp:
            return
        self.last_stamp = current_time

        out_msg = ModelStates()
        transforms = []
        
        for i, name in enumerate(data.name):
            # 检查模型名字是否是棋子
            if any(name.startswith(p) for p in self.prefixes):
                out_msg.name.append(name)
                out_msg.pose.append(data.pose[i])
                out_msg.twist.append(data.twist[i])

                # 构建 TF 变换
                t = TransformStamped()
                t.header.stamp = current_time
                t.header.frame_id = "world" # 参考系：世界坐标系
                t.child_frame_id = name     # 子坐标系：棋子名字 (例如 "pion1")
                
                # 直接使用 Gazebo 返回的实时位姿
                t.transform.translation.x = data.pose[i].position.x
                t.transform.translation.y = data.pose[i].position.y
                t.transform.translation.z = data.pose[i].position.z
                t.transform.rotation = data.pose[i].orientation
                
                transforms.append(t)

        # 批量发布 TF 和话题
        if out_msg.name:
            self.pieces_pub.publish(out_msg)
            self.tf_broadcaster.sendTransform(transforms)

if __name__ == '__main__':
    try:
        ChessPiecesManager()
    except rospy.ROSInterruptException:
        pass