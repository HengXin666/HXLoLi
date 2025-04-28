# 001.第一个Node节点
## 1.1 包和结点

简单地说, 一个包里面就含有一个或多个节点

## 1.2 创建包

命令格式

```sh
catkin_create_pkg <包名> <依赖项>
```

示例:
```sh
~/catkin_ws/src$ catkin_create_pkg ssr_pkg rospy roscpp std_msgs
```

可以看到, 创建了一个文件夹和 cmake 和 包信息.xml(类似于java的Maven)
```C++
loli@HengXin-ROG-PRIME:~/catkin_ws/src/ssr_pkg$ ll
total 28
drwxr-xr-x 3 loli loli 4096 Sep 19 15:27 include/
drwxr-xr-x 2 loli loli 4096 Sep 19 15:27 src/
-rw-r--r-- 1 loli loli 7052 Sep 19 15:27 CMakeLists.txt
-rw-r--r-- 1 loli loli 2860 Sep 19 15:27 package.xml
```

- 注: 我们可以使用

```sh
roscd <包名>
```

跳转到系统安装的该ROS包的位置, 例如:

```sh
~/catkin_ws/src/ssr_pkg$ roscd roscpp
/opt/ros/noetic/share/roscpp$
```

## 1.3 第一个ROS程序

在 src 里面 创建 `chao_node.cpp` 写入:

```C++
#include <ros/ros.h>

int main(int argc, char const *argv[]) {
    printf("Hello 世界!\n");
    return 0;
}
```

然后在cmake末尾添加:

```cmake
add_executable(chao_node src/chao_node.cpp)
```

编译后, 启动`roscore`, 就可以通过

```sh
rosrun <包名> <节点>
```

如:

```sh
rosrun ssr_pkg chao_node

# 输出
Hello 世界!
```

## 1.4 升级一下

```C++
#include <ros/ros.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "chao_node");
    printf("Hello 世界!\n");
    int i = 0;
    while (true) { // 如果 ctrl + c 无法退出, 就换成 ros::ok()
        printf("输出 %d\n", ++i);
    }
    return 0;
}
```

cmake 需要链接:

```cmake
add_executable(chao_node src/chao_node.cpp)
target_link_libraries(chao_node
  ${catkin_LIBRARIES}
)
```