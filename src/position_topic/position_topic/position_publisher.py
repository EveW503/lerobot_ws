import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class NodePublisher(Node):
   def __init__(self, name: str):
      super().__init__(name)
      self.get_logger().info("目标位置发布节点已启动 (仅位置模式)")

      self.declare_parameter("topic_name", "/target_pose")
      # 建议默认改成 base_link，视你的URDF而定
      self.declare_parameter("frame_id", "base") 
      self.declare_parameter("publish_rate_hz", 1.0)
      self.declare_parameter("position", [0.30, 0.0, 0.25])
      # 删除了 orientation 参数的声明

      topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
      publish_rate_hz = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

      self.command_publisher_ = self.create_publisher(PoseStamped, topic_name, 10)
      self.timer = self.create_timer(1.0 / max(publish_rate_hz, 1e-3), self.publish_target_pose)

   def publish_target_pose(self) -> None:
      frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
      position = self.get_parameter("position").get_parameter_value().double_array_value

      if len(position) != 3:
         self.get_logger().error("参数长度错误: position应为3维")
         return

      msg = PoseStamped()
      msg.header.stamp = self.get_clock().now().to_msg()
      msg.header.frame_id = frame_id
      
      # 1. 赋予你指定的位置
      msg.pose.position.x = float(position[0])
      msg.pose.position.y = float(position[1])
      msg.pose.position.z = float(position[2])
      
      # 2. 赋予默认的单位四元数（占位用，后续在C++中会被忽略）
      msg.pose.orientation.x = 0.0
      msg.pose.orientation.y = 0.0
      msg.pose.orientation.z = 0.0
      msg.pose.orientation.w = 1.0

      self.command_publisher_.publish(msg)
      self.get_logger().info(
         f"已发布目标位置: p=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f}) [姿态已设为随意]"
      )

def main(args=None):
   rclpy.init(args=args)
   node = NodePublisher("target_pose_publisher")
   try:
      rclpy.spin(node)
   except KeyboardInterrupt:
      pass
   finally:
      node.destroy_node()
      rclpy.shutdown()

if __name__ == "__main__":
   main()