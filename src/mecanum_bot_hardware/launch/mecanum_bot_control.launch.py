from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Arguments
    can_interface = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface name'
    )
    
    node_id_base = DeclareLaunchArgument(
        'node_id_base',
        default_value='1',
        description='Base node ID for CANopen motors'
    )
    
    use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation'
    )

    # Hardware interface node
    hardware_node = Node(
        package='mecanum_bot_hardware',
        executable='mecanum_bot_system',
        name='mecanum_bot_system',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'node_id_base': LaunchConfiguration('node_id_base'),
            'update_rate': 100,
            'joints': [
                'front_left_wheel_joint',
                'front_right_wheel_joint',
                'rear_left_wheel_joint',
                'rear_right_wheel_joint'
            ]
        }]
    )
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[PathJoinSubstitution([
            FindPackageShare('mecanum_bot_hardware'),
            'config',
            'mecanum_bot_controllers.yaml'
        ])],
        output='screen'
    )
    
    # Joint state broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Diff drive controller
    diff_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command([
                'xacro ',
                PathJoinSubstitution([
                    FindPackageShare('mecanum_bot_hardware'),
                    'urdf',
                    'mecanum_bot.urdf'
                ])
            ])
        }]
    )
    
    return LaunchDescription([
        can_interface,
        node_id_base,
        use_sim,
        hardware_node,
        controller_manager,
        joint_state_broadcaster,
        diff_drive_controller,
        robot_state_publisher,
    ])