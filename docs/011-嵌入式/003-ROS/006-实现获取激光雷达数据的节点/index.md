# 006.实现获取激光雷达数据的节点
## 1. ROS系统中的激光雷达消息包格式

- 用到的包: https://index.ros.org/p/sensor_msgs/github-ros-common_msgs/#noetic

- 激光雷达消息包格式: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html

```py
# 单次扫描来自平面激光测距仪
#
# 如果您有其他具有不同行为的测距设备（例如声纳阵列），请查找或创建不同的消息，因为应用程序将对该数据做出相当激光特定的假设。

Header header            # 时间戳在头部表示扫描中第一个光线的采集时间。
                         #
                         # 在 frame_id 坐标系中，角度围绕正 Z 轴测量（逆时针，如果 Z 轴向上）
                         # 零角度沿着 X 轴向前。

float32 angle_min        # 扫描的起始角度 [弧度]
float32 angle_max        # 扫描的结束角度 [弧度]
float32 angle_increment  # 测量之间的角距 [弧度]

float32 time_increment   # 测量之间的时间 [秒] - 如果您的扫描仪在移动，这将用于插值计算 3D 点的位置。
float32 scan_time        # 扫描之间的时间 [秒]

float32 range_min        # 最小范围值 [米]
float32 range_max        # 最大范围值 [米]

float32[] ranges         # 距离数据 [米]（注意：值小于 range_min 或大于 range_max 应被丢弃）
float32[] intensities    # 强度数据 [设备特定单位]。如果您的设备不提供强度，请保持数组为空。
```

查看一次完整的数据: (启动仿真, 然后查看话题`rostopic echo /scan`)

```C++
header: 
  seq: 64
  stamp: 
    secs: 113
    nsecs: 834000000
  frame_id: "laser"
angle_min: -3.141590118408203
angle_max: 3.141590118408203
angle_increment: 0.017501894384622574
time_increment: 0.0
scan_time: 0.0
range_min: 0.23999999463558197
range_max: 6.0
ranges: [inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 2.618407964706421, 2.601997137069702, 2.6176862716674805, 2.6235759258270264, 2.612292766571045, 2.624370574951172, 2.6019246578216553, 2.5831236839294434, 2.585765838623047, 2.620004415512085, 2.592240333557129, 2.6039998531341553, 2.601938009262085, 2.5927653312683105, 2.6017141342163086, 2.6190075874328613, 2.645850896835327, 2.628106117248535, 2.6349587440490723, 2.648963689804077, 2.649444103240967, 2.669956684112549, 2.6730563640594482, 2.706454038619995, 2.698719024658203, 2.71858286857605, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf]
intensities: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

## 2. C++实现获取激光雷达数据

- 用到的开源工程: `wpr_simulation`

1. 新建一个包

```sh
catkin_create_pkg lidar_pkg roscpp rospy sensor_msgs
```

2. 代码

```C++
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

void laiar_cb(const sensor_msgs::LaserScan& msg) {
    float fMidDist = msg.ranges[180];
    ROS_INFO("前方测距值为 ranges[180] = %.4f 米", fMidDist);
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "lidar_node");

    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, laiar_cb);

    ros::spin();

    return 0;
}
```
