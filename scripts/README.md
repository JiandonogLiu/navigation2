
### 将xacro转换成urdf文件
```
xacro src/apa_description/urdf/model.xacro > src/apa_description/urdf/model.urdf
```

### 将urdf转换成sdf文件
```
gz sdf -p src/apa_description/urdf/model. > src/apa_description/urdf/model.sdf
```

### gazebo加载sdf文件
```
gazebo --verbose src/apa_description/urdf/model.sdf
```

### 编译功能包
```
colcon build --packages-select apa_description
```

### 解决依赖编译功能包
colcon build --packages-up-to  apa_navigation2

### 运行键盘控制节点
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 打印控制话题
```
ros2 topic echo /cmd_vel
```

### 保存地图
```
cd src/apa_cartographer/map
ros2 run nav2_map_server map_saver_cli -t map -f fishbot_map
```
