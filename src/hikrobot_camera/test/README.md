# 测试目录

你可以在这里添加调试过程中需要的测试，并在 `CMakeLists.txt` 和 `package.xml` 中配置对应依赖。

目前没有预置测试。完成驱动后，请实际连接相机，检查图像发布、参数设置和断线重连是否正常。
# 安装
1. 安装ROS2 Humble
2. 厂商安装：HIKROBOT MVS SDK(Linux)

# 编译与运行
## 编译
```bash
colcon build --symlink-install --packages-select hikrobot_camera
source /opt/ros/humble/setup.zsh
source install/setup.zsh
```
## 运行
```bash
ros2 launch hikrobot_camera camera.launch.py
```
## 查看
```bash
source install/setup.zsh
ros2 run rqt_image_view rqt_image_view
```
