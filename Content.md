# 新增ROS2发送节点
尝试1：由于没有官方的pycharm和ROS2调试教程，按网上非官方教程调试无果，放弃了！
尝试2：按网上非官方VScode和ROS2调试教程尝试无果，放弃了！
现采用构建一个新节点，封装成ROS2包，launch运行，命令行输出比较不方便的方法进行调试。


进入/home/thu/ros2_workspace/src/point_cloud_infer
创建一个新的节点文件（将检测框传输到OBU）`bounding_box_client.py`

一定注意在配置文件/home/thu/ros2_workspace/src/point_cloud_infer/setup.py
中添加该节点，不然系统找不到该节点
```
'bounding_box_client = point_cloud_infer.bounding_box_client:main',
```
！！！注意最后面有，！！！


在工作区的目录下`ros2_workspace`，将新节点文件封装为包

```
colcon build --packages-select point_cloud_infer
```

创建一个launch文件，用于配置和启动多个节点。
在文件夹`ros2_workspace/src/point_cloud_infer/launch`下，创建新的launch文件`bounding_box_client_launch.py`

进入到工作区的目录下`ros2_workspace`，source安装文件
```
. install/setup.bash
```

运行launch文件
```
ros2 launch point_cloud_infer bounding_box_client_launch.py pcd_path:=/home/thu/Downloads/2021_08_23_21_47_19/243 engine_path:=/home/thu/Downloads/PointPillarNet/checkpoint_epoch_80_fp32_v2.engine rate:=2
```

运行之前的launch文件，只实现检测框显示
```
ros2 launch point_cloud_infer point_cloud_infer_launch.py pcd_path:=/home/thu/Downloads/2021_08_23_21_47_19/243 engine_path:=/home/thu/Downloads/PointPillarNet/checkpoint_epoch_80_fp32_v2.engine rate:=2
```

经过测试无误，接下来考虑实现其功能

先看一下要传输的框的数据，先放一些数据的截图
