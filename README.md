# 简介
此仓库为大创项目的巡检车存储仓库，内容为小车仿真模型以及相关的nav2导航代码

# 拉取仓库
```
git clone https://github.com/qin343531/car_ros2.git
```

## 拉取navaigation2
```
cd car_ros2/src/
git clone https://github.com/ros-planning/navigation2.git -b humble
```

## 编译
开始编译
```
cd ~/car_ros2
colcon build
```

# 脚本启动
首先要运行`ares_description/launch/qinbot_dis.launch.py`脚本
```
ros2 launch ares_description qinbot_dis.launch.py
```
这个会启动gazebo仿真页面，里面有小车仿真模型与地图

然后运行`qinbot_navigation2/launch/navigation2.launch.py`脚本
```
ros2 launch navigation2 turtlebot3_navigation2.launch.py
```
这个会启动导航功能，rviz界面
之后在rviz2中进行导航即可
![](./images/nav2.png)