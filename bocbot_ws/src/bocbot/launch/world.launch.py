import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    robot_name = 'bocbot'
    world_file_name = 'boc_office.world'

    # full path to urdf and world file
    world = os.path.join(get_package_share_directory(robot_name), 'worlds', world_file_name)
    urdf = os.path.join(get_package_share_directory(robot_name), 'urdf', 'bocbot.urdf')
    
    # create and return launch description object
    return LaunchDescription([
        # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'),

        # tell gazebo to spawn your robot in the world by using the spawn_entity node
        Node(
            package='gazebo_ros',
            node_executable='spawn_entity.py',
            arguments=['-entity', 'bocbot', '-file', urdf],
            output='screen'),

        # launch our custom WASD teleop in a new xterm window so it runs simultaneously
        ExecuteProcess(
            cmd=['xterm', '-title', 'Robot Controller (WASD)', '-e', 'python3', os.path.join(get_package_share_directory(robot_name), 'src', 'teleop_wasd.py')],
            output='screen'
        ),
    ])