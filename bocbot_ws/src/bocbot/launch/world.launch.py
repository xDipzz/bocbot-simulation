import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    robot_name = 'bocbot'
    world_file_name = 'boc_office.world'

    # full path to urdf and world file
    world = os.path.join(get_package_share_directory(robot_name), 'worlds', world_file_name)
    urdf = os.path.join(get_package_share_directory(robot_name), 'urdf', 'bocbot.urdf')
    teleop_script = os.path.join(get_package_share_directory(robot_name), 'src', 'teleop_wasd.py')

    spawn_entity = Node(
        package='gazebo_ros',
        node_executable='spawn_entity.py',
        arguments=['-entity', 'bocbot', '-file', urdf],
        output='screen')

    teleop = ExecuteProcess(
        cmd=['xterm', '-title', 'Robot Controller (WASD)', '-e', 'python3', teleop_script],
        output='screen'
    )

    def _on_spawn_exit(event, context):
        """Only start teleop if spawn_entity succeeded (exit code 0)."""
        if event.returncode == 0:
            return [teleop]
        return [LogInfo(msg='spawn_entity failed (exit {}) — teleop not started'.format(
            event.returncode))]

    # create and return launch description object
    return LaunchDescription([
        # start gazebo, notice we are using libgazebo_ros_factory.so instead of libgazebo_ros_init.so
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so', '-s', 'libgazebo_ros_state.so'],
            output='screen'),

        # tell gazebo to spawn your robot in the world by using the spawn_entity node
        spawn_entity,

        # Start teleop only if spawn_entity exits successfully.
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_entity,
                on_exit=_on_spawn_exit,
            )
        ),
    ])
