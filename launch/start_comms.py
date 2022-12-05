from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, IfCondition

def generate_launch_description():
    count = LaunchConfiguration('count')
    record = LaunchConfiguration('record')
    # record_flag = ''
    # if(record == "true"):
    #     record_flag = '-a'
    
    declare_count = DeclareLaunchArgument(
        'count',
        default_value='1',
        description='Count value')

    declare_record = DeclareLaunchArgument(
        'record',
        default_value='false',
        description='record all topics')
    print("record arg: ", record)
    return LaunchDescription([
        declare_count, 
        declare_record,
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-a']
        , condition=IfCondition(PythonExpression([record, '== 1']))
        ),
        
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