#!/bin/bash

# 检查并创建src文件夹
if [ ! -d src ]; then
    echo "Creating src directory..."
    mkdir src
fi

# 获取当前目录名称
CURRENT_DIR=$(basename "$PWD")


# 默认依赖项
DEFAULT_DEPENDENCIES=("rclcpp" "std_msgs")

# 获取用户输入的功能包名称
read -p "Enter package name: " PKG_NAME

# 获取用户输入的附加依赖项列表
ADDITIONAL_DEPENDENCIES=()
while true; do
  read -p "Enter an additional dependency (or leave empty to finish): " DEP
  if [ -z "$DEP" ]; then
    break
  fi
  ADDITIONAL_DEPENDENCIES+=($DEP)
done

# 构建完整的依赖项列表
DEPENDENCIES=("${DEFAULT_DEPENDENCIES[@]}" "${ADDITIONAL_DEPENDENCIES[@]}")

# 构建依赖项字符串
DEPENDENCIES_STR=$(IFS=" " ; echo "${DEPENDENCIES[*]}")

# 在src目录下创建并编译功能包
cd src
ros2 pkg create $PKG_NAME --build-type ament_cmake --dependencies $DEPENDENCIES_STR --node-name $PKG_NAME
cd ..
colcon build --packages-select $PKG_NAME