#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ai_msgs.msg import PerceptionTargets

class BodyPoseSubscriber(Node):
    def __init__(self):
        super().__init__('body_pose_subscriber')
        self.subscription = self.create_subscription(
            PerceptionTargets,
            '/hobot_mono2d_body_detection',
            self.listener_callback,
            10)
        self.get_logger().info('Body Pose Subscriber node has been started')

    def listener_callback(self, msg):
        self.get_logger().info(f"Received frame: {msg.header.frame_id}, FPS: {msg.fps}")
        
        # 检查是否有检测到的目标
        if len(msg.targets) > 0:
            self.get_logger().info(f"Detected {len(msg.targets)} targets")
            
            for i, target in enumerate(msg.targets):
                self.get_logger().info(f"Target {i+1} - Type: {target.type}, Track ID: {target.track_id}")
                
                # 处理人体关键点
                for point_group in target.points:
                    if point_group.type == "body_kps" and len(point_group.point) > 0:
                        self.get_logger().info(f"Body keypoints detected: {len(point_group.point)} points")
                        
                        # 定义人体关键点的名称
                        keypoint_names = [
                            "Nose", "Neck", "Right Shoulder", "Right Elbow", "Right Wrist",
                            "Left Shoulder", "Left Elbow", "Left Wrist", "Right Hip", "Right Knee",
                            "Right Ankle", "Left Hip", "Left Knee", "Left Ankle", "Right Eye",
                            "Left Eye", "Right Ear", "Left Ear"
                        ]
                        
                        # 打印每个关键点的位置和置信度
                        for j, (point, conf) in enumerate(zip(point_group.point, point_group.confidence)):
                            if j < len(keypoint_names):
                                name = keypoint_names[j]
                            else:
                                name = f"Point {j}"
                                
                            self.get_logger().info(f"  {name}: ({point.x:.2f}, {point.y:.2f}) - Confidence: {conf:.4f}")
                
                # 处理ROI区域
                for roi in target.rois:
                    self.get_logger().info(f"  ROI Type: {roi.type}, Position: x={roi.rect.x_offset}, y={roi.rect.y_offset}, "
                                          f"width={roi.rect.width}, height={roi.rect.height}")
        else:
            self.get_logger().info("No targets detected")
        
        self.get_logger().info("-" * 50)

def main(args=None):
    rclpy.init(args=args)
    node = BodyPoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# #!/usr/bin/env python3
# # 指定Python解释器路径
# import rclpy  # ROS2 Python客户端库
# from rclpy.node import Node  # ROS2节点基类
# from ai_msgs.msg import PerceptionTargets  # AI消息类型，用于感知目标数据
# import time
# import math
# from scservo_sdk import *  # 舵机控制库

# class BodyPoseSubscriber(Node):
#     """人体姿态订阅者节点类，继承自ROS2的Node基类"""
    
#     def __init__(self):
#         """初始化节点"""
#         super().__init__('body_pose_subscriber')
        
#         # 创建订阅者，订阅人体检测话题
#         self.subscription = self.create_subscription(
#             PerceptionTargets,
#             '/hobot_mono2d_body_detection',
#             self.listener_callback,
#             10)
        
#         # 初始化舵机控制
#         self.portHandler = PortHandler('/dev/ttyACM0')
#         self.packetHandler = sms_sts(self.portHandler)
        
#         # 舵机ID和运动参数
#         self.SCS_ID = 1
#         self.SPEED = 2750
#         self.ACCEL = 250
        
#         # 打开串口
#         if not self.portHandler.openPort():
#             print("❌ 打开串口端口失败")
#             return
        
#         # 设置波特率
#         if not self.portHandler.setBaudRate(1000000):
#             print("❌ 设置波特率失败")
#             self.portHandler.closePort()
#             return
            
#         print("✅ 舵机连接成功！")
        
#         # 初始舵机位置
#         self.current_angle = 90  # 初始角度为90度
#         self.move_servo(self.current_angle)

#     def angle_to_position(self, angle):
#         """将角度转换为舵机位置值"""
#         angle = max(0, min(180, angle))  # 限制在0-180范围内
#         return int((angle / 180) * 4095)

#     def move_servo(self, angle):
#         """移动舵机到指定角度"""
#         position = self.angle_to_position(angle)
        
#         # 清除之前的参数
#         self.packetHandler.groupSyncWrite.clearParam()
        
#         # 添加新参数并发送
#         if self.packetHandler.SyncWritePosEx(self.SCS_ID, position, self.SPEED, self.ACCEL):
#             result = self.packetHandler.groupSyncWrite.txPacket()
#             self.packetHandler.groupSyncWrite.clearParam()
            
#             if result == COMM_SUCCESS:
#                 self.current_angle = angle
#                 return True
        
#         return False

#     def calculate_head_tilt_angle(self, left_shoulder, right_shoulder):
#         """计算头部倾斜角度"""
#         # 计算两肩之间的向量
#         dx = right_shoulder.x - left_shoulder.x
#         dy = right_shoulder.y - left_shoulder.y
        
#         # 计算与水平线的夹角
#         angle = math.atan2(dy, dx) * (180 / math.pi)
        
#         # 将角度限制在-45到45度之间（合理的头部倾斜范围）
#         angle = max(-45, min(45, angle))
        
#         return angle

#     def listener_callback(self, msg):
#         """消息回调函数，处理接收到的人体检测数据"""
        
#         # 检查是否检测到目标
#         if len(msg.targets) > 0:
#             # 只处理第一个检测到的人体
#             target = msg.targets[0]
            
#             # 提取肩膀位置
#             left_shoulder = None
#             right_shoulder = None
            
#             for point_group in target.points:
#                 if point_group.type == "body_kps" and len(point_group.point) > 0:
#                     for j, (point, conf) in enumerate(zip(point_group.point, point_group.confidence)):
#                         if j == 5 and conf > 0.5:  # 左肩，置信度大于0.5
#                             left_shoulder = point
#                         elif j == 2 and conf > 0.5:  # 右肩，置信度大于0.5
#                             right_shoulder = point
            
#             # 如果找到了肩膀位置
#             if left_shoulder and right_shoulder:
#                 # 计算头部倾斜角度
#                 tilt_angle = self.calculate_head_tilt_angle(left_shoulder, right_shoulder)
                
#                 # 将倾斜角度映射到舵机角度（0-180度）
#                 # 向右倾斜（正角度）-> 舵机角度减小
#                 # 向左倾斜（负角度）-> 舵机角度增大
#                 servo_angle = 90 - tilt_angle
#                 servo_angle = max(0, min(180, servo_angle))
                
#                 # 输出头部倾斜角度
#                 if tilt_angle > 0:
#                     print(f"头部向右倾斜: {tilt_angle:.1f}°")
#                 elif tilt_angle < 0:
#                     print(f"头部向左倾斜: {abs(tilt_angle):.1f}°")
#                 else:
#                     print("头部正直")
                
#                 # 移动舵机到新的角度
#                 self.move_servo(servo_angle)

# def main(args=None):
#     """主函数"""
#     # 初始化ROS2
#     rclpy.init(args=args)
    
#     # 创建节点实例
#     node = BodyPoseSubscriber()
    
#     try:
#         # 启动节点，开始接收和处理消息
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         # 捕获Ctrl+C中断信号
#         print("\n程序退出")
#     finally:
#         # 清理资源
#         node.destroy_node()  # 销毁节点
#         rclpy.shutdown()     # 关闭ROS2

# # 程序入口点
# if __name__ == '__main__':
#     main()



