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






