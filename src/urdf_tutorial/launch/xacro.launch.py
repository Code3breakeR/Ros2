import launch
from launch.substitutions import LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='urdf_tutorial').find('urdf_tutorial')
    default_model_path = os.path.join(pkg_share, 'urdf/r2d2.urdf.xml')
    default_rviz_config_path = os.path.join(pkg_share, 'urdf/r2d2.rviz')
                                 
    print("urdf_file_name -------------- : {}".format(default_model_path))

    with open(default_model_path, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name ='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        arguments=[default_model_path],
    )
    
    publisher_node = launch_ros.actions.Node(
        package='urdf_tutorial',
        executable='r2d2',
        name='r2d2',
        output='screen',
        arguments=['-d', LaunchConfiguration('publisher')],
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='publisher', default_value='True',
                                              description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                             description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                             description='Absolute path to rviz config file'),
        
        publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])