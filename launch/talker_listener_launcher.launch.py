from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Function to generate launch description
def generate_launch_description():
    
    # Declare frequency argument
    freq_arg = DeclareLaunchArgument(
        'freq',
        default_value='2.0',  # Default frequency if none is provided
        description='Frequency for the publisher node in Hz'
    )
    
    # Declare record_bag argument to control ros bag recording
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='false',  # Default to not recording if not specified
        description='Enable or disable ros bag recording'
    )
    
    # create handle for publisher node
    minimal_publisher_node = Node(
        package='beginner_tutorials',
        executable='talker',
        name='minimal_publisher',
        output='screen',
        parameters=[{'freq': LaunchConfiguration('freq')}],
    )
    
    # create handle for subscriber node
    minimal_subscriber_node = Node(
        package='beginner_tutorials',
        executable='listener',
        name='minimal_subscriber',
        output='screen',
    )
    
    # create handle for conditional recording of ros bag
    def conditional_rosbag_record(context):
        if LaunchConfiguration('record_bag').perform(context) == 'true':
            return [
                ExecuteProcess(
                    cmd=['ros2', 'bag', 'record', '--all'],
                    output='screen',
                    cwd='src/my_beginner_tutorials/bag_files'  # Set the working directory
                )
            ]
        return []

    # return launch description
    return LaunchDescription([
        freq_arg,
        record_bag_arg,
        minimal_publisher_node,
        minimal_subscriber_node,
        OpaqueFunction(function=conditional_rosbag_record)
    ])
