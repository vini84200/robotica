from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import Shutdown
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    ## ***** Parametros ******
    map_name_arg= DeclareLaunchArgument('map_name', default_value='denseMap')
    map_server_config_path= DeclareLaunchArgument('final_map_config', default_value=[FindPackageShare('tp4').find('tp4'), '/maps/', LaunchConfiguration('map_name'), '.yaml'])

    ## ***** Caminhos de diretorios ******
    pkg_share = FindPackageShare('phi_aria').find('phi_aria')
    urdf_dir = os.path.join(pkg_share, 'description/urdf/')
    urdf_file = os.path.join(urdf_dir, 'p3dx.urdf.xacro')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rviz_file = FindPackageShare('tp4').find('tp4') + '/config/p3dx.rviz'
    
    aria_port = 'port:=localhost'                # simulation
    # aria_port = 'port:=192.168.1.11:10002'     # real robot

    ## ***** Nodos *****
    robot_state_publisher_node = Node(
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': False}
            ],
        output = 'screen'
    )

    joint_state_publisher_node = Node(
        package = 'joint_state_publisher',
        executable = 'joint_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': False}
            ],
        output = 'screen'
    )

    # Nodo que conecta ao robo usando a biblioteca Aria
    aria_node = Node(
        package='phi_aria', executable='phi_p3dx',
        output='screen',
        arguments=[
        '--ros-args', '-p', 'publish_aria_lasers:=true', '-p', aria_port
        ]
    )

    # Nodo para publicação do mapa
    map_server_node = Node(
        package='nav2_map_server', executable='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('final_map_config')}]
    )

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  # https://github.com/ros2/launch/issues/188
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )

    tf_map = Node(
        package='tf2_ros', executable='static_transform_publisher', 
        arguments = "0 0 0 0 0 0 map odom".split(' ')
    )
   
    # Nodo de visualizacao
    rviz_node = Node(
        package='rviz2', executable='rviz2',
        output='log',  
        on_exit = Shutdown(),
        arguments = ['-d', rviz_file],
    )

    return LaunchDescription([
        map_name_arg,
        map_server_config_path,
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        map_server_node,
        start_lifecycle_manager_cmd,
        tf_map,
        aria_node,
        rviz_node,
    ])
