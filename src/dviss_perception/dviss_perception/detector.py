import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 1. 订阅摄像头
        self.subscription = self.create_subscription(
            Image,
            '/perception/image_raw',  # 对应 usb_cam 的话题
            self.image_callback,
            10)
        
        # 2. 发布处理后的图像 (用于调试显示)
        self.img_pub = self.create_publisher(Image, '/perception/yolo_result', 10)
        
        # 3. 发布机械臂目标坐标
        self.target_pub = self.create_publisher(Point, '/arm/target_point', 10)
        
        self.bridge = CvBridge()
        
        # 加载 YOLO 模型 (会自动下载到当前目录)
        self.get_logger().info('正在加载 YOLOv8 模型 (首次运行可能需要下载)...')
        self.model = YOLO("yolov8n.pt") 
        self.get_logger().info('✅ 模型加载完毕')

    def image_callback(self, msg):
        # 转换 ROS 图像 -> OpenCV 格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return

        # 执行推理
        results = self.model(cv_image, verbose=False)
        
        # 解析结果
        for r in results:
            boxes = r.boxes
            for box in boxes:
                # 获取边界框坐标
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = box.conf[0]
                cls = int(box.cls[0])
                label = self.model.names[cls]

                # 🎯 核心逻辑：我们只抓 "cup", "bottle", "apple" (示例)
                # 你可以在这里过滤你想抓的东西
                if conf > 0.5:
                    # 计算中心点 (Pixel Frame)
                    u_center = int((x1 + x2) / 2)
                    v_center = int((y1 + y2) / 2)

                    # 画框框 (Visual Debug)
                    cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                    cv2.circle(cv_image, (u_center, v_center), 5, (0, 0, 255), -1)
                    cv2.putText(cv_image, f"{label} {conf:.2f}", (int(x1), int(y1)-10), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # 🧮 坐标转换 (Pixel -> Real World)
                    # 这是一个非常粗糙的映射，仅供逻辑测试
                    # 假设画面中心是 (0,0)，每个像素代表 0.5mm
                    img_h, img_w, _ = cv_image.shape
                    
                    real_x = (u_center - img_w / 2) * 0.5  # 简单的比例映射
                    real_y = (v_center - img_h / 2) * 0.5
                    real_z = 0.0 # 假设物体在桌面上

                    # 发布目标给机械臂
                    target_msg = Point()
                    target_msg.x = float(real_x)
                    target_msg.y = float(real_y)
                    target_msg.z = float(real_z)
                    self.target_pub.publish(target_msg)
                    
                    self.get_logger().info(f'发现了 {label}! 发送抓取坐标: ({real_x:.1f}, {real_y:.1f})')

        # 发布画了框的图
        self.img_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()