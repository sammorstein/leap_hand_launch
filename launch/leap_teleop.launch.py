import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():

    urdf = os.path.join(get_package_share_directory('leap_hand_densetact'), 'model/leap_right/robot.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
	
	Node(
            package='leap_hand_densetact',
            executable='leaphand_node.py',
            name='leaphand_node',
            output='screen',
            parameters=[
                {'kP': 800.0},
                {'kI': 0.0},
                {'kD': 200.0},
                {'curr_lim': 500.0}
            ]
        ),
	
        # Node(
        # package='leap_hand_densetact',
        # executable='teleoperation_node.py',
        # name='teleoperation_node',
        # output='screen'
        # ),

        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     name='robot_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            
        # ),

        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='joint_state_publisher',
        #     output='screen',
        #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
            
        # ),

        # Node(
        #         package='rviz2',
        #         executable='rviz2',
        #         name='rviz2',
        #         arguments=[
                    
        #         ],
        #     ),        

        

    ])
