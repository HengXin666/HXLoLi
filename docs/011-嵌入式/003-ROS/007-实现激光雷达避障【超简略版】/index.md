# 007.实现激光雷达避障(超简略版)

- 原理:
    - `"/scan"`--得到雷达数据-->`程序处理`--发送-->`"/cmd_vel"`(进行移动)
 
代码, 直接在[006.实现获取激光雷达数据的节点](../006-实现获取激光雷达数据的节点/index.md)基础上进行修改了

- 代码:

```C++
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// 全局变量, 方便在`laiar_cb`回调函数里面使用
ros::Publisher pub_vel;

void laiar_cb(const sensor_msgs::LaserScan& msg) {
    float fMidDist = msg.ranges[180];
    ROS_INFO("前方测距值为 ranges[180] = %.4f 米", fMidDist);

    geometry_msgs::Twist vel_cmd;

/**
 * 需要考虑机器人自身的轮廓, 这里只是粗略的逻辑, 某些情况下可能会一直原地转圈
 */

    if (msg.ranges[165] < 1.f || msg.ranges[195] < 1.f || fMidDist < 1.f) {
        vel_cmd.angular.z = 0.5;
    } else {
        vel_cmd.linear.x = 0.5;
    }
    
    pub_vel.publish(vel_cmd);
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "lidar_node");

    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/scan", 10, laiar_cb);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::spin();

    return 0;
}
```