# 008.实现IMU航向锁定
## 1. 认识IMU数据
- 用到的包: http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/Imu.html


```py
# 这是用于存储IMU数据的消息
#
# 加速度应以 m/s² 为单位（而不是以 g 为单位），旋转速度应以 rad/sec 为单位
#
# 如果测量的协方差已知，应填写该值（如果您仅知道每个测量值的方差，例如来自数据表，则只需将其放置在对角线上）
# 如果协方差矩阵全为零，将被解释为“协方差未知”，使用数据时必须假设或从其他来源获取协方差
#
# 如果对某个数据元素没有估计（例如，您的IMU不产生方向估计），请将相关协方差矩阵的第一个元素设置为 -1
# 如果您正在解释此消息，请检查每个协方差矩阵的第一个元素是否为 -1，并忽略相关的估计值。

Header header

geometry_msgs/Quaternion orientation
float64[9] orientation_covariance # 关于x、y、z轴的行主序

geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance # 关于x、y、z轴的行主序

geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance # 行主序x、y、z
```

这几种数据结构的区别在于它们表示的物理量和对应的协方差:

1. **orientation**: 表示物体的方向，使用四元数表示。`orientation_covariance` 是方向估计的不确定性（协方差）。

2. **angular_velocity**: 表示物体的角速度，使用三维向量表示。`angular_velocity_covariance` 是角速度测量的不确定性。

3. **linear_acceleration**: 表示物体的矢量加速度，使用三维向量表示。`linear_acceleration_covariance` 是加速度测量的不确定性。

总的来说，它们分别描述了物体的方向、旋转速度和线性加速度，以及各自的测量精度。

其中 orientation 的类型是`geometry_msgs::Quaternion`:

```C++
float64 x
float64 y
float64 z
float64 w
```

为四元数, 以解决使用欧拉角时候出现的万向锁的问题 (实际使用的时候)

## 2. C++实现IMU数据获取

### 2.1 需要用到的话题

`imu/data_raw` (sensor_msgs/lmu)
- 加速度计输出的矢量加速度和陀螺仪输出的旋转角速度

`imu/data` (sensor_msgs/lmu)
- `/imu/data_raw`的数据再加上融合后的四元数姿态描述

`imu/mag` (sensor_msgs/MagneticField)
- 磁强计输出磁强数据。

一般订阅第二个话题

### 2.2 代码实现

- 建包
```sh
catkin_create_pkg imu_pkg roscpp rospy sensor_msgs
```

- 代码
 
```C++
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <math.h>

void imu_cb(const sensor_msgs::Imu& msg) {
    if (msg.orientation_covariance[0] < 0) // 数据是否有效
        return;
    
    // 将四元素转换为欧拉角
    tf::Quaternion quaternion {
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    };

    // 用于存放 四元素转换的 欧拉角
    double roll,  // 翻滚 
           pitch, // 俯仰
           yaw;   // 航向角

    // 先转换为 3x3矩阵对象; getRPY 转换为 欧拉角 (参数是传出参数) (单位是弧度)
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    // 将弧度转为角度
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;

    ROS_INFO("翻滚 = %.0f, 俯仰 = %.0f, 航向角 = %.0f", roll, pitch, yaw);
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "imu_node");

    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/imu/data", 10, imu_cb);
    ros::spin();

    return 0;
}
```

## 3. C++实现IMU航向锁定

- 直接在`2`的基础上修改:

```C++
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <math.h>

// 全局变量, 方便在`laiar_cb`回调函数里面使用
ros::Publisher pub_vel;

void imu_cb(const sensor_msgs::Imu& msg) {
    if (msg.orientation_covariance[0] < 0) // 数据是否有效
        return;
    
    // 将四元素转换为欧拉角
    tf::Quaternion quaternion {
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w
    };

    // 用于存放 四元素转换的 欧拉角
    double roll,  // 翻滚 
           pitch, // 俯仰
           yaw;   // 航向角

    // 先转换为 3x3矩阵对象; getRPY 转换为 欧拉角 (参数是传出参数) (单位是弧度)
    tf::Matrix3x3(quaternion).getRPY(roll, pitch, yaw);

    // 将弧度转为角度
    roll = roll * 180 / M_PI;
    pitch = pitch * 180 / M_PI;
    yaw = yaw * 180 / M_PI;

    ROS_INFO("翻滚 = %.0f, 俯仰 = %.0f, 航向角 = %.0f", roll, pitch, yaw);

    // === IMU航向锁定 ===
    double target_yaw = 90; // 目标角度
    double diff_angle = target_yaw - yaw; // 差值角度

    geometry_msgs::Twist vel_cmd;
    vel_cmd.angular.z = diff_angle * 0.01; // 比例系数 (pid的p)
    vel_cmd.linear.x = 0.1;
    pub_vel.publish(vel_cmd);
}

int main(int argc, char *argv[]) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "imu_node");

    ros::NodeHandle nh;
    ros::Subscriber lidar_sub = nh.subscribe("/imu/data", 10, imu_cb);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::spin();

    return 0;
}
```
