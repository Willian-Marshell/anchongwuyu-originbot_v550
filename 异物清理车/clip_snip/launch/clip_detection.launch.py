from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='clip_snip',
            executable='clip_detector',
            name='clip_detector',
            output='screen'
        ),
        Node(
            package='clip_snip',
            executable='motion_controller',
            name='motion_controller',
            output='screen',
            parameters=['/userdata/dev_ws/src/originbot/clip_snip/config/pid_params.yaml']  # ¼ÓÔØPID²ÎÊý
        )
    ])