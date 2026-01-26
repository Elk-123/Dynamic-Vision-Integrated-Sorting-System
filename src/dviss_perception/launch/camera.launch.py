import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取包路径
    pkg_name = 'dviss_perception'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 配置文件路径
    config_file = os.path.join(pkg_share, 'config', 'camera_params.yaml')

    # 定义 USB Cam 节点
    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam',
        namespace='perception',  # 加上命名空间，方便管理
        parameters=[config_file],
        output='screen'
    )

    # 定义 rqt_image_view 节点 (可选，方便调试时自动弹出)
    # 在生产环境中通常会注释掉
    rqt_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='image_viewer',
        arguments=['/perception/image_raw']
    )

    return LaunchDescription([
        usb_cam_node,
        rqt_node
    ])