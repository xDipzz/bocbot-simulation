import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = 'bocbot'
    world_file_name = 'boc_office.world'

    # full path to urdf and world file
    world = os.path.join(get_package_share_directory(robot_name), 'worlds', world_file_name)
    urdf = os.path.join(get_package_share_directory(robot_name), 'urdf', 'bocbot.urdf')

    spawn_entity = Node(
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        arguments=['-entity', 'bocbot', '-file', urdf],
        output='screen')

    autonomous_tour = Node(
        package='bocbot',
        node_executable='autonomous_tour.py',
        name='autonomous_tour',
        output='screen'
    )

    # create and return launch description object
    return LaunchDescription([
        # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_state.so'],
            output='screen'),

        # tell gazebo to spawn your robot in the world by using the spawn_entity node
        spawn_entity,

        # Start the autonomous tour after spawn_entity exits
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=[autonomous_tour],
            )
        ),
    ])
