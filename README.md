# Cartographer_lifelong

主旨在于提高机器人在环境变化后的定位稳定性.

开发中...

# 1.Build

```bash
git clone https://github.com/ZYCheng1002/cartographer_lifelong.git
cd cartographer_lifelong
colcon build
```

# 2.Run

## 2.1 ros1 bag转ros2

```bash
sudo pip install rosbags # 如果不增加sudo,可能会安装到.config目录下,导致无法找到rosbags-convert命令
sudo pip install --upgrade rosbags
# 将 ROS1 的 rosbag 转换为 ROS2 格式： 
rosbags-convert --src ros1.bag  --dst rosbag_dir
```

## 2.2 backpack 2d datasets

```bash
ros2 launch cartographer_ros demo_backpack_2d.launch.py bag_filename:=rosbag_dir
```

## 2.3 localization

# 3.Todo List

- [ ] 保留建图时的图节点和边信息;
- [ ] 增加cross check 判断大范围动态物体;
- [ ] 匹配score判断优化--基于几何信息的适应性得分调整;
- [ ] 去除3D SLAM

# 4.Acknowledgements

