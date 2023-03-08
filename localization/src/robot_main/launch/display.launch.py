import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os
from glob import glob

def generate_launch_description():



    #-----------------------path declare------------------------------
    # main pkg
    pkg_share = launch_ros.substitutions.FindPackageShare(package='robot_main').find('robot_main')
    
    # robot description file path and rviz path 
    default_model_path = os.path.join(pkg_share, 'src/description/robot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    # laser driver path
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lidar_uart_ros2','lsm10_p.yaml')
    # map file path
    map_file = os.path.join(get_package_share_directory('robot_main'), 'map', 'EBEEL3.yaml')
    #world_path=os.path.join(pkg_share, 'world/my_world.sdf')
    
    # navigation2 file path
    nav2_yaml = os.path.join(get_package_share_directory('robot_main'), 'config', 'amcl_config.yaml')
    #-----------------------------------------------------------------



    #------------------------Node declare------------------------------
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        #condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    lsliadar_init = LifecycleNode(
        package='lslidar_driver',
        executable='lslidar_driver_node',
        name='lslidar_driver_node',		
        output='screen',
        emulate_tty=True,
        namespace='',
        parameters=[driver_dir],
    )
    imu_init = launch_ros.actions.Node(
        package='imu',
        executable='imu_node',
        name='imu_node',
    )
    mapserver_node = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': False}, 
                    {'yaml_filename':map_file}]
    )
    amcl_node = launch_ros.actions.Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_yaml]
    )
    lifecycle_node = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server', 'amcl']}]  
    )
    '''
    spawn_entity = launch_ros.actions.Node(
    	package='gazebo_ros', 
    	executable='spawn_entity.py',
        arguments=['-entity', 'sam_bot', '-topic', 'robot_description'],
        output='screen'
    )
    '''

    odom_node = launch_ros.actions.Node(
        package='difftf_py',
        executable='diff_tf',
        name='diff_tf',
    )
    odom_trans = launch_ros.actions.Node(
        package='difftf_py',
        executable='odom_publisher',
        name='odom_publisher',
    )
    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf.yaml'), {'use_sim_time': False}]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        #arguments=['-d', LaunchConfiguration('rvizconfig')],
    )


    return launch.LaunchDescription([
    # Some parameter you can add when you launch
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        
    #robot description
        joint_state_publisher_node,
        robot_state_publisher_node,

        #spawn_entity,
    #localization and navigation
        mapserver_node,
        amcl_node,
        lifecycle_node,
    #odom driver and transmit
        odom_node,
        odom_trans,
    #data fusion(imu and laser) driver
        robot_localization_node,
    #imu and laser driver
        imu_init,
        lsliadar_init,
    #visualization
        rviz_node,
    ])
