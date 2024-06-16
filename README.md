# rm_buff

RoboMaster能量机关自动瞄准算法模块

## 写在前面

本项目基于 [FaterYU/rm_vision (github.com)](https://github.com/FaterYU/rm_vision) 框架开发，与 [FaterYU/rm_auto_aim (github.com)](https://github.com/FaterYU/rm_auto_aim) 可共存运行，使用 [FaterYU/rm_serial_driver (github.com)](https://github.com/FaterYU/rm_serial_driver) 进行串口通信。

<img src="docs/rm_vision.svg" alt="rm_vision" width="200" height="200">

该项目为 [rm_vision](https://github.com/FaterYU/rm_vision) 的子模块

若有帮助请Star这个项目

[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

## 文档

[2024EnergyGain](./docs/2024EnergyGain.md)

## 效果演示

TBD

## 使用指南

### 使用rm_vision镜像

拉取镜像

```shell
docker pull ghcr.io/fateryu/rm_vision:latest
```

构建开发容器

```shell
docker run -it --name rv_devel \
--privileged --network host \
-v /dev:/dev -v $HOME/.ros:/root/.ros -v ws:/ros_ws \
ghcr.io/fateryu/rm_vision:latest \
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

构建运行容器

```shell
docker run -it --name rv_runtime \
--privileged --network host --restart always \
-v /dev:/dev -v $HOME/.ros:/root/.ros -v ws:/ros_ws \
ghcr.io/fateryu/rm_vision:latest \
ros2 launch rm_vision_bringup vision_bringup.launch.py
```

### 在rm_vision中启动

1. 将本模块 `main` 分支拉取到 `ros_ws/src/` 目录下
2. 确保包含 `rm_vison` `rm_serial_driver` `rm_gimbal_description` 模块
3. `colcon build --symlink-install`
4. `source ./install/setup.zsh`
5. `ros2 launch rm_vision_bringup vision_bringup.launch.py`

### 模块独立启动

1. 将本模块 `main` 分支拉取到 `ros_ws/src/` 目录下
2. `colcon build --symlink-install`
3. `source ./install/setup.zsh`
4. `ros2 launch rm_buff_bringup rm_buff_bringup.launch.py`

### 模型训练

[FaterYU/rm_buff at train (github.com)](https://github.com/FaterYU/rm_buff/tree/train) 

### 下位机运算示例

[buff_calculate](./example/buff_calculate.c)

### 可视化

```shell
sudo apt install ros-$ROS_DISTRO-foxglove-bridge
source ./install/setup.zsh
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```

## 项目包结构

- [buff_detector](./buff_detector)

  订阅相机参数及图像流进行能量机关扇叶的识别并解算三维位置，输出识别到的能量机关扇叶在输入frame下的三维位置 (一般是以相机光心为原点的相机坐标系)

- [buff_tracker](./buff_tracker)

  订阅识别节点发布的能量机关扇叶三维位置及机器人的坐标转换信息，将能量机关扇叶三维位置变换到指定惯性系（一般是以云台中心为原点，IMU 上电时的 Yaw 朝向为 X 轴的惯性系）下，然后将能量机关扇叶变化到能量机关主体下，并将能量机关送入跟踪器中，输出跟踪能量机关在指定惯性系下的状态，根据任务类型判断大小能量机关，若为小能量机关则直接输出能量机关状态，若为大能量机关则继续将目标角速度序列进行高斯牛顿迭代法，拟合出运动速度方程的参数输出

- [buff_interfaces](./buff_interfaces)

  定义了识别节点和处理节点的接口以及定义了用于 Debug 的信息

- [rm_buff_brintup](./rm_buff_bringup)

  包含启动识别节点和处理节点的默认参数文件及 launch 文件

