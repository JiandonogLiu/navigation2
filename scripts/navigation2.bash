#!/bin/bash

# 检查是否提供了文件夹路径参数
if [ $# -ne 1 ]; then
  echo "输入正确的文件夹路径"
  exit 1
fi

# 将src后的功能包名作为参数传入
package_name=$(basename "$1")

xacro src/$package_name/urdf/model.xacro >src/$package_name/urdf/model.urdf

colcon build --packages-select $package_name

# 使用 pgrep 命令查找 Gazebo 进程的 PID
gazebo_pid=$(pgrep -f 'gzserver|gzclient')

# 判断 PID 是否为空
if [ -z "$gazebo_pid" ]; then
  echo "未找到 Gazebo 进程。"
else
  # 循环杀死每个 Gazebo 进程
  for pid in $gazebo_pid; do
    echo "杀死 Gazebo 进程 $pid"
    kill -9 $pid
  done
fi

ros2 launch $package_name navigation2_launch.py
