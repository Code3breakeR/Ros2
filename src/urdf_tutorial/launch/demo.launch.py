import os

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    model = LaunchConfiguration('use_sim_time', default='false')
    urdf_file_name = 'xacro/svaya_aiu_6_v2.urdf.xacro'

    xacro = os.path.join(
        get_package_share_directory('urdf_tutorial'),
        urdf_file_name)

    print("urdf_file_name : {}".format(xacro))
    
    base_path = os.path.realpath(get_package_share_directory('urdf_tutorial')) 
    rviz_path=base_path+'/rviz2/urdf.rviz'
    print("rviz_path : {}".format(rviz_path))

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        # ExecuteProcess(
        #     cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
        #     output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': model}],
            arguments=[xacro]),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', str(rviz_path)]),
        Node(
            package='urdf_tutorial',
            executable='ros2talker',
            name='ros2talker',
            output='screen'),
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_entity.py',
        #     name='urdf_spawner',
        #     output='screen',
        #     arguments=["-topic", "/robot_description", "-entity", "cam_bot"]),
        
    ])
