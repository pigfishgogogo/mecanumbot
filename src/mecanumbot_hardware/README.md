# 在一个终端运行节点
 ros2 run mecanumbot_hardware mecanumbot_system 
 
# 在另一个终端发送命令
# 启用电机
ros2 topic pub /enable_motor std_msgs/msg/Bool "{data: true}" -1

# 设置速度
ros2 topic pub /target_velocity std_msgs/msg/Int32 "{data: 500}" -1

# 查看状态
ros2 topic echo /motor_status
ros2 topic echo /actual_velocity

# 发送字符串命令
ros2 topic pub /motor_command std_msgs/msg/String "{data: 'start'}" -1
ros2 topic pub /motor_command std_msgs/msg/String "{data: 'zero'}" -1
ros2 topic pub /motor_command std_msgs/msg/String "{data: 'stop'}" -1
ros2 topic pub /motor_command std_msgs/msg/String "{data: 'status'}" -1


# launch 启动
ros2 launch mecanumbot_hardware mecanumbot_control.launch.py 