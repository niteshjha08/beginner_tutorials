from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

process_rosbag_record = ExecuteProcess(
      cmd=['ros2', 'bag', 'record', '-a'],
)
def generate_launch_description():
    count = LaunchConfiguration('count')
    declare_count = DeclareLaunchArgument(
        'count',
        default_value='1',
        description='Count value')

    return LaunchDescription([
        declare_count, 
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a']),
        
        Node(
            package='cpp_pubsub',
            executable='talker',
            name='talker',
            parameters=[{'count':count}]
        ),
        Node(
            package='cpp_pubsub',
            executable='listener',
            name='listener'
        )
    ])