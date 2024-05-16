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

ros2 launch $package_name rviz_launch.py
