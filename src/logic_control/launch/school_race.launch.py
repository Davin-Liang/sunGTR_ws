from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch.substitutions import TextSubstitution
from ament_index_python.packages import get_package_prefix

# 1. 在 root 目录下运行该 launch
# 2. 在 root 目录下得有 /obstacle_config 目录 # TODO:
# 3. 已在 dnn_node_example 中固定使用 yolov5 # TODO:
# 4. 已给 dnn_example_image_width、dnn_example_image_height 赋官方的默认值，无需在命令行中赋值

def generate_launch_description():
    # 拷贝 config 中文件
    cp_cmd = "cp -r /opt/tros/lib/racing_obstacle_detection_yolo/config/ ./obstacle_config/"
    os.system(cp_cmd)

    image_width_launch_arg = DeclareLaunchArgument(
        "dnn_example_image_width", default_value=TextSubstitution(text="480") 
    )
    image_height_launch_arg = DeclareLaunchArgument(
        "dnn_example_image_height", default_value=TextSubstitution(text="272")
    )

    # 相机节点
    mipi_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_localization'), 'launch'), # TODO:
            '/ekf_x1_x3_launch.py']) # TODO:
    )

    # 赛道检测节点
    racing_tracking_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('racing_track_detection_resnet'), 'launch'),
            '/racing_track_detection_resnet_simulation.launch.py'])
    )

    # 赛道障碍物检测节点
    obstacle_detection_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('racing_obstacle_detection_yolo'), 'launch'),
            '/racing_obstacle_detection_yolo_simulation.launch.py'])
    )

    # 警戒线检测节点 
    warning_type_detection_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('dnn_node_example'), 'launch'),
            '/dnn_node_example.launch.py']),
        launch_arguments={
            'dnn_example_image_width': LaunchConfiguration('dnn_example_image_width'),
            'dnn_example_image_height': LaunchConfiguration('dnn_example_image_height'),
            }.items()
    )

    # 停车控制节点
    parking_control_node = Node(
        package='logic_control',
        executable='school_race',
    )

    # 底盘控制节点
    originbot_bringup_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('originbot_bringup'), 'launch'),
            '/originbot.launch.py'])
    )

    # 寻线控制节点
    racing_control_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('racing_control'), 'launch'),
            '/racing_control.launch.py'])
    )

    return LaunchDescription([
        image_width_launch_arg,
        image_height_launch_arg,

        mipi_node,                      # 开启相机
        racing_tracking_node,           # 开启赛道检测
        obstacle_detection_node,        # 开启障碍物检测
        warning_type_detection_node,    # 开启警戒线检测
        parking_control_node,           # 开启停车控制
        originbot_bringup_node,         # 开启底盘控制
        racing_control_node             # 开启寻线控制
    ])
        