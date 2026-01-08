from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('mecanumbot_hardware'),
            'urdf',
            'mecanumbot.urdf'
        ])
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # 加载控制器配置文件
    controller_config = PathJoinSubstitution([
        FindPackageShare('mecanumbot_hardware'),
        'config',
        # 'controllers.yaml'
        'mecanumbot_controllers.yaml'
    ])
    
    # 控制器管理器节点
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_config],
        output='screen',
        emulate_tty=True,
    )
    
    # 延迟启动控制器，确保硬件已初始化
    from launch.actions import TimerAction
    
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        emulate_tty=True,
    )
    
    mecanum_drive_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mecanum_drive_controller'],
        output='screen',
        emulate_tty=True,
    )
    
    # 机器人状态发布器
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # 先启动控制器管理器和机器人状态发布器
        controller_manager,
        robot_state_publisher,
        joint_state_broadcaster,
        mecanum_drive_controller,
    ])


# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
# import os

# def generate_launch_description():
#     # 加载URDF文件
#     robot_description_content = Command([
#         'xacro ',
#         PathJoinSubstitution([
#             FindPackageShare('mecanumbot_hardware'),
#             'urdf',
#             'mecanumbot.urdf'
#         ])
#     ])
    
#     robot_description = {'robot_description': robot_description_content}
    
#     # # 硬件系统节点
#     # hardware_node = Node(
#     #     package='mecanumbot_hardware',
#     #     executable='mecanumbot_system',
#     #     name='mecanumbot_system',
#     #     output='screen',
#     #     parameters=[robot_description]  # 传入机器人描述
#     # )
    
#     # 控制器管理器节点
#     controller_manager = Node(
#         package='controller_manager',
#         executable='ros2_control_node',
#         parameters=[robot_description],  # 控制器管理器也需要机器人描述
#         output='screen'
#     )
    
#     # 启动其他控制器
#     joint_state_broadcaster = Node(
#         package='controller_manager',
#         executable='spawner',
#         arguments=['joint_state_broadcaster'],
#         output='screen'
#     )
    
#     robot_state_publisher = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         name='robot_state_publisher',
#         output='screen',
#         parameters=[robot_description]
#     )
    
#     return LaunchDescription([
#         # hardware_node,
#         controller_manager,
#         joint_state_broadcaster,
#         robot_state_publisher,
#     ])