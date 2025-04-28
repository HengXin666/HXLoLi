# 011.SLAM与Hector_Mapping
## 1. SLAM

- https://www.bilibili.com/video/BV1FW4y1M7PV/

SLAM（Simultaneous Localization and Mapping）是一个用于移动机器人和无人驾驶车辆的技术，它允许设备在未知环境中同时进行自我定位和地图构建。通过传感器（如激光雷达、相机等）收集环境信息，SLAM算法能够实时更新机器人的位置和环境地图，从而实现自主导航。

omg! 太复杂了, 不过! 異議あり! 导包即可!

## 2. Hector_Mapping

- 用到的包: https://index.ros.org/p/hector_mapping/github-tu-darmstadt-ros-pkg-hector_slam/#noetic

### 2.1 订阅主题

扫描 （[sensor\_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html)）

- SLAM 系统使用的激光扫描。

syscommand （[std\_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)）

- syscommand。如果字符串等于 “reset”，则地图和机器人姿势将重置为其初始状态。

[hector_mapping - ROS Wiki](https://wiki.ros.org/hector_mapping)

### 2.2 已发布的主题

map_metadata ([nav\_msgs/MapMetaData](http://docs.ros.org/en/api/nav_msgs/html/msg/MapMetaData.html))

- 从此主题获取 map 数据，该主题是 latched 的，并定期更新。

mpa ([nav\_msgs/OccupancyGrid](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

- 从此主题获取 map 数据，该主题是 latched 的，并定期更新

slam_out_pose ([geometry\_msgs/PoseStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseStamped.html))

- 无协方差的估计机器人姿势

poseupdate ([geometry\_msgs/PoseWithCovarianceStamped](http://docs.ros.org/en/api/geometry_msgs/html/msg/PoseWithCovarianceStamped.html))

- 具有高斯不确定性估计的机器人姿势

> 一般订阅第二个话题(`/map`)

### 2.3 安装依赖

```sh
sudo apt install ros-noetic-hector-mapping
```
### 2.4 启动

```sh
roslaunch wpr_simulation wpb_stage_slam.launch
```

```sh
rosrun hector_mapping hector_mapping
```

```sh
rosrun rviz rviz
```

```sh
rosrun rqt_robot_steering rqt_robot_steering
```

## 3. 通过launch文件启动Hector_Mapping的建图功能

1. 新建包

```sh
catkin_create_pkg slam_pkg rospy roscpp std_msgs
```

2. 创建文件夹`launch`, 以及`.launch`文件, 并且写入以下内容: (把上面四条指令翻译为`.launch`配置)

```xml
<launch>
    <!-- roslaunch wpr_simulation wpb_stage_slam.launch  -->
    <include file="$(find wpr_simulation)/launch/wpb_stage_slam.launch"/>

    <!-- rosrun hector_mapping hector_mapping -->
    <node pkg="hector_mapping" type="hector_mapping" name="hector_mapping"/>

    <!-- rosrun rviz rviz -->
    <!-- <node pkg="rviz" type="rviz" name="rviz"/> -->

    <!-- 或者: 从配置文件启动 rviz -->
    <node pkg="rviz" type="rviz" name="rviz" args="-d $(find slam_pkg)/rviz/slam.rviz"/>

    <!-- rosrun rqt_robot_steering rqt_robot_steering -->
    <node pkg="rqt_robot_steering" type="rqt_robot_steering" name="rqt_robot_steering"/>
</launch>
```

3. 编译, 运行

```sh
roslaunch slam_pkg hector.launch
```

## 4. Hector_Mapping建图的参数设置

> ~base\_frame （ 字符串 ，默认值：base\_link）
> 
> - 机器人的基架名称。这是用于定位和转换激光扫描数据的框架。
> 
> ~map\_frame （ 字符串 ，默认值：map\_link）
> 
> - 地图框的名称。
> 
> ~odom\_frame （ 字符串 ，默认值：odom）
> 
> - odom 帧的名称。
> 
> ~map\_resolution （ double ，默认值：0.025）
> 
> - 地图分辨率 \[m\]。这是网格单元边缘的长度。
> 
> ~map\_size （ int ，默认值：1024）
> 
> - 映射的大小 \[每个轴的单元格数\]。地图是正方形的，并且具有 （map\_size \* map\_size） 个网格单元格。
> 
> ~map\_start\_x （ double ，默认值：0.5）
> 
> - /map 帧的原点 \[0.0， 1.0\] 在 x 轴上相对于网格映射的位置。0.5 位于中间。
> 
> ~map\_start\_y （ double ，默认值：0.5）
> 
> - /map 帧的原点 \[0.0， 1.0\] 在 y 轴上相对于网格映射的位置。0.5 位于中间。
> 
> ~map\_update\_distance\_thresh （ double ，默认值：0.4）
> 
> - 执行映射更新的阈值 \[m\]。在地图更新发生之前，平台必须以米为单位行驶这么远，或者经历自上次更新以来 map\_update\_angle\_thresh 参数所描述的角度变化。
> 
> ~map\_update\_angle\_thresh （ double ，默认值：0.9）
> 
> - 执行映射更新的阈值 \[rad\]。在地图更新发生之前，平台必须经历自上次更新以来 map\_update\_distance\_thresh 参数指定的行程参数所描述的角度变化。
> 
> ~map\_pub\_period （ double ，默认值：2.0）
> 
> - 地图发布周期 \[s\]。
> 
> ~map\_multi\_res\_levels （ int ，默认值：3）
> 
> - 贴图多分辨率网格级别的数量。
> 
> ~update\_factor\_free （ double ，默认值：0.4）
> 
> - 用于更新 \[0.0， 1.0\] 范围内自由单元格的 map update 修饰符。值 0.5 表示没有变化。
> 
> ~update\_factor\_occupied （ 双 精度，默认值：0.9）
> 
> - 用于更新范围 \[0.0， 1.0\] 中占用单元格的 map update 修饰符。值 0.5 表示没有变化。
> 
> ~laser\_min\_dist （ double ，默认值：0.4）
> 
> - 系统要使用的激光扫描端点的最小距离 \[m\]。将忽略小于此值的 Scan 端点。
> 
> ~laser\_max\_dist （ double ，默认值：30.0）
> 
> - 系统要使用的激光扫描端点的最大距离 \[m\]。扫描比此值更远的端点将被忽略。
> 
> ~laser\_z\_min\_value （ double ，默认值：-1.0）
> 
> - 系统要使用的激光扫描端点相对于激光扫描仪框架的最小高度 \[m\]。小于此值的 Scan 端点将被忽略。
> 
> ~laser\_z\_max\_value （ double ，默认值：1.0）
> 
> - 系统要使用的激光扫描端点相对于激光扫描仪框架的最大高度 \[m\]。高于此值的 Scan 端点将被忽略。
> 
> ~pub\_map\_odom\_transform （ bool ，默认值：true）
> 
> - 确定系统是否应发布 map->odom 转换。
> 
> ~output\_timing （ bool ，默认值：false）
> 
> - 输出定时信息，以便通过 ROS\_INFO 处理每个激光扫描。
> 
> ~scan\_subscriber\_queue\_size （ int ，默认值：5）
> 
> - 扫描订户的队列大小。如果日志文件以比实时更快的速度播放到 hector\_mapping，则应将其设置为较高的值（例如 50）。
> 
> ~pub\_map\_scanmatch\_transform （ bool ，默认值： true）
> 
> - 确定是否应将 scanmatcher 到 map 转换发布到 tf。帧名称由 'tf\_map\_scanmatch\_transform\_frame\_name' 参数确定。
> 
> ~tf\_map\_scanmatch\_transform\_frame\_name （ 字符串 ，默认值：scanmatcher\_frame）
> 
> - 将 scanmatcher 发布到 map transform 时的帧名称，如前面的参数中所述。

例如: 把之前的配置文件改为:

```xml
<node pkg="hector_mapping" type="hector_mapping" name="hector_mapping" output="screen">
    <param name="map_update_distance_thresh" value="0.1"/>
    <param name="map_update_angle_thresh" value="0.1"/>
    <param name="map_pub_period" value="0.1"/>
</node>
```

可以发现更新频率明显变快了!