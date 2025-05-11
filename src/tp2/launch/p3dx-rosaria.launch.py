from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    ## ***** Caminhos de diretorios ******
    pkg_share = FindPackageShare('phi_aria').find('phi_aria')
    urdf_dir = os.path.join(pkg_share, 'description/urdf/')
    urdf_file = os.path.join(urdf_dir, 'p3dx.urdf.xacro')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()
    rviz_file = FindPackageShare('tp2').find('tp2') + '/config/p3dx.rviz'
    
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
   
    # Node de visualizacao
    rviz_node = Node(
        package='rviz2', executable='rviz2',
        output='log',  
        on_exit = Shutdown(),
        arguments = ['-d', rviz_file],
    )

    return LaunchDescription([
        # Nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        aria_node,
        rviz_node,
    ])
