import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import time

class ArmController(Node):
    def __init__(self):
        super().__init__('arm_controller')
        
        # === 核心接口 ===
        # 无论以后用什么机械臂，都监听这个 Topic
        self.subscription = self.create_subscription(
            Point,
            '/arm/target_point',
            self.listener_callback,
            10)
        
        self.get_logger().info('🤖 机械臂抽象层已启动 - 等待目标指令...')
        self.is_moving = False

    def listener_callback(self, msg):
        if self.is_moving:
            self.get_logger().warn(f'⚠️ 机械臂忙碌中，忽略指令: [{msg.x:.2f}, {msg.y:.2f}]')
            return

        self.is_moving = True
        self.get_logger().info(f'📨 收到指令: 移动至 X={msg.x:.2f}, Y={msg.y:.2f}, Z={msg.z:.2f}')
        
        # === 模拟硬件运动 (Simulation) ===
        # 当真机到来时，这里替换为串口写入代码 (e.g., serial.write(...))
        time.sleep(2.0) # 假装移动了2秒
        
        self.get_logger().info(f'✅ 到达目标: [{msg.x:.2f}, {msg.y:.2f}] - 抓取动作执行中...')
        time.sleep(1.0) # 假装抓取
        self.is_moving = False
        self.get_logger().info('💤 动作完成，这就绪')

def main(args=None):
    rclpy.init(args=args)
    node = ArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()