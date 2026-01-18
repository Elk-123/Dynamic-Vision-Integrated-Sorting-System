
我们将采用 **"Python 大脑 + C++ 小脑"** 的混合架构：
*   **Perception (视觉)**: Python (利用 PyTorch/YOLO 生态的便捷性)。
*   **Control (驱动/运动)**: C++ (利用 `ros2_control` 及其 Realtime 约束，确保毫秒级响应)。

---

### 3.1 粮草先行：环境与依赖 (Environment Setup)

在真机环境，OS 的实时补丁（Preempt-RT）是加分项，但标准内核对于非高速场景通常足够。

**基础环境清单**：
*   **OS**: Ubuntu 22.04 LTS (Jammy Jellyfish) - *ROS 2 Humble 的 Tier 1 平台*。
*   **Middleware**: ROS 2 Humble Hawksbill。
*   **Compiler**: GCC 11.4+ / CMake 3.22+。
*   **Hardware SDKs**:
    *   相机驱动：`librealsense2` (如果是 Intel RealSense) 或 `libuvc` (通用 USB 相机)。
    *   机械臂 SDK：厂家提供的 C/C++ 动态链接库 (.so) 或 通信协议文档。

**依赖锁定 (Dependency Injection)**：
请在你的 Shell 中执行以下环境自检与安装指令：

```bash
# 1. 确保 ROS2 基础环境与构建工具
sudo apt update && sudo apt install -y \
  ros-humble-desktop \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit \
  ros-humble-cv-bridge \
  python3-colcon-common-extensions \
  python3-pip \
  git

# 2. Python 核心依赖 (用于视觉节点)
pip3 install ultralytics numpy opencv-python pyserial
# 注意：ultralytics 是 YOLOv8 官方库，生产环境建议导出为 ONNX 或 TensorRT 运行以加速
```

---

### 3.2 安营扎寨：工程目录结构 (Project Layout)

这是一个标准的 ROS 2 `colcon` 工作空间结构。我们将创建 4 个核心 Package。

**执行以下 Shell 脚本一键生成骨架：**

```bash
mkdir -p ~/visarm_ws/src
cd ~/visarm_ws/src

# 1. 视觉感知包 (Python)
ros2 pkg create --build-type ament_python visarm_perception \
  --dependencies rclpy sensor_msgs cv_bridge geometry_msgs

# 2. 硬件接口包 (C++ Core - 性能关键)
ros2 pkg create --build-type ament_cmake visarm_driver \
  --dependencies rclcpp hardware_interface pluginlib rclcpp_lifecycle

# 3. 描述与配置包 (URDF & MoveIt Config)
ros2 pkg create --build-type ament_cmake visarm_description

# 4. 启动入口包
ros2 pkg create --build-type ament_cmake visarm_bringup

# 建立标准目录结构
mkdir -p visarm_perception/visarm_perception/weights
mkdir -p visarm_driver/include/visarm_driver
mkdir -p visarm_description/{urdf,meshes,launch}
mkdir -p visarm_bringup/{launch,config}

echo "✅ 项目骨架已部署。当前位置: $(pwd)"
```

**目录解析**：
*   `visarm_driver` (C++): 这里我们将实现 `SystemInterface`，这是连接你真实机械臂的**生死桥梁**。
*   `visarm_perception`: 放置 YOLO 模型和推理代码。
*   `visarm_description`: 存放机械臂的 URDF 模型（必须精准，否则 MoveIt 规划会撞墙）。

---

### 3.3 阵地攻坚：核心代码交付 (Core Implementation)

由于你是“真机实战”，最核心的风险在于**硬件通信**。如果 Python 发送指令太慢或 GC 导致卡顿，机械臂可能会抖动。因此，我们优先实现 **C++ 硬件接口**。

#### ⚔️ 核心模块 A：C++ 硬件抽象层 (Hardware Interface)

我们将基于 `ros2_control` 实现一个通用的 6轴机械臂接口。这是企业级机器人开发的标准做法。

**文件路径**: `~/visarm_ws/src/visarm_driver/include/visarm_driver/visarm_hardware_interface.hpp`

```cpp
#ifndef VISARM_DRIVER__VISARM_HARDWARE_INTERFACE_HPP_
#define VISARM_DRIVER__VISARM_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace visarm_driver
{
// 继承自 SystemInterface，这是 ROS2 Control 的标准插件接口
class VisArmSystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VisArmSystemHardware)

  // 1. 初始化：读取 URDF 中的参数（如串口号、波特率）
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  // 2. 导出状态接口：告诉 ROS 当前机械臂在哪 (Position, Velocity)
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // 3. 导出命令接口：接收 ROS 的控制指令 (Position Command)
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // 4. 激活硬件（打开串口/TCP）
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 5. 停用硬件（关闭串口/安全回零）
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  // 6. 核心循环：读取硬件状态 (Read from Robot)
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // 7. 核心循环：写入硬件指令 (Write to Robot)
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // 模拟通信用的缓冲区，实战中替换为你的 SDK 调用
  std::vector<double> hw_commands_;
  std::vector<double> hw_states_position_;
  std::vector<double> hw_states_velocity_;

  // ⚠️ 宗师提示：这里应包含你的底层通信类实例，例如:
  // SerialPort serial_port_;
};

}  // namespace visarm_driver

#endif  // VISARM_DRIVER__VISARM_HARDWARE_INTERFACE_HPP_
```

**文件路径**: `~/visarm_ws/src/visarm_driver/src/visarm_hardware_interface.cpp`

*(这里仅展示核心的 Read/Write 逻辑，这是性能优化的关键点)*

```cpp
#include "visarm_driver/visarm_hardware_interface.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <vector>

namespace visarm_driver
{

// ... (on_init, export_interfaces 省略，按标准模板填充) ...

hardware_interface::return_type VisArmSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 🛡️ 架构师防御性编程：在此处添加 try-catch 块
  // 如果读取超时，不要直接崩溃，而是记录 Warning 并尝试重连或保持上一帧状态
  try {
    // TODO: 调用你的真实硬件 SDK，例如: robot_sdk.get_joint_angles()
    
    // 模拟数据回环（仅用于测试链路，接入真机时删除此逻辑）
    for (size_t i = 0; i < hw_states_position_.size(); ++i) {
      // 假设硬件完美响应
      hw_states_position_[i] = hw_commands_[i]; 
      // 实际开发中，这里应该是: hw_states_position_[i] = receive_from_serial(i);
    }
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("VisArmSystemHardware"), "Communication Error: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VisArmSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 🛡️ 安全检查：指令幅度是否过大？(防止飞车)
  // 如果当前指令与上一帧差异过大，强制限幅
  
  try {
    // TODO: 将 hw_commands_ 数组打包并通过串口/TCP 发送
    // robot_sdk.send_joints(hw_commands_);
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("VisArmSystemHardware"), "Write Error: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace visarm_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  visarm_driver::VisArmSystemHardware, hardware_interface::SystemInterface)
```

#### ⚔️ 核心模块 B：视觉感知节点 (Python YOLOv8)

对于视觉部分，我们使用 Python，因为 C++ 处理 Tensor 变换极其痛苦且 ROI 低。

**文件路径**: `~/visarm_ws/src/visarm_perception/visarm_perception/detector_node.py`

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        
        # 🛡️ 架构师模式：参数化配置，不要硬编码模型路径
        self.declare_parameter('model_path', 'yolo11n.pt') 
        self.declare_parameter('confidence_threshold', 0.5)
        
        # 加载模型 (自动下载或使用本地)
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.get_logger().info(f'Loading YOLO model: {model_path}...')
        self.model = YOLO(model_path)
        
        self.bridge = CvBridge()
        
        # 订阅相机图像
        self.create_subscription(Image, '/camera/color/image_raw', self.image_callback, 10)
        
        # 发布检测结果 (3D Pose)
        self.pose_pub = self.create_publisher(PoseArray, '/detected_objects', 10)
        
        # 调试用：发布画了框的图
        self.debug_pub = self.create_publisher(Image, '/perception/debug_image', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'CV Bridge Error: {e}')
            return

        # 推理
        results = self.model(cv_image, verbose=False)
        
        pose_array = PoseArray()
        pose_array.header = msg.header # 继承相机的时间戳和Frame ID，这对 TF 至关重要
        
        # 解析结果
        for result in results:
            boxes = result.boxes
            for box in boxes:
                if box.conf[0] < self.get_parameter('confidence_threshold').get_parameter_value().double_value:
                    continue
                
                # 2D 像素坐标
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)
                
                # ⚠️ 难点：2D -> 3D 映射
                # 在真实项目中，这里必须结合 /camera/aligned_depth_to_color/image_raw 深度图
                # 暂时使用伪代码逻辑占位，后续必须在此处注入深度查询逻辑
                z_depth = 0.5 # 假设物体在 0.5m 处 (TODO: Replace with real depth query)
                
                # 简单的针孔相机模型反投影 (需内参 fx, fy, cx, cy)
                # X = (u - cx) * Z / fx
                # Y = (v - cy) * Z / fy
                
                pose = Pose()
                pose.position.x = (cx - 320) * z_depth / 600.0 # 示例参数
                pose.position.y = (cy - 240) * z_depth / 600.0
                pose.position.z = z_depth
                pose_array.poses.append(pose)
                
                # 画框 (Debug)
                cv2.rectangle(cv_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)

        self.pose_pub.publish(pose_array)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

---

### 3.4 实弹演习：构建与验证 (Build & Verify)

在开始写真实的串口通信逻辑前，必须先编译通过，验证架构通路。

1.  **编译项目**:
    ```bash
    cd ~/visarm_ws
    colcon build --symlink-install
    source install/setup.bash
    ```

2.  **单元测试 (Sanity Check)**:
    验证 C++ 插件是否能被 ROS2 系统识别。
    ```bash
    ros2 run visarm_driver visarm_driver_node --ros-args -p use_dummy:=true
    # 注意：你需要编写一个简单的 main 函数或者使用 ros2_control_node 来加载插件
    ```
    *（注：通常我们通过 Launch 文件加载 `ros2_control_node`，并传入 URDF）*

---

### 🔮 下一步指令：接入真实硬件

现在的代码仅仅是“空架子”（Skeleton）。要让它动起来，我需要你提供**硬件的具体信息**，这是**无法回避**的物理约束：

1.  **机械臂型号**：是基于串口的（如 Arduino/STM32 自制），还是基于网口的（如 UR, xArm）？**通信协议是什么？**
2.  **相机型号**：RealSense D435？Orbbec Astra？还是普通的 USB WebCam？

**请提供硬件详情，我将为你生成具体的 `Protocol Implementation` 代码。**