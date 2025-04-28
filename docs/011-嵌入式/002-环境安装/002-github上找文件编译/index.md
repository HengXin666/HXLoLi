https://www.bilibili.com/video/BV1dV4y1u758/ 学习视频

仓库
- https://github.com/6-robot/wpr_simulation

注意:

- 安装依赖这一步, 每一条安装指令都要运行多次, 不然漏了就会编译不过

```sh
cd ~/catkin_ws/src/wpr_simulation/scripts
./install_for_noetic.sh
cd ~/catkin_ws/src/wpb_home/wpb_home_bringup/scripts
./install_for_noetic.sh
cd ~/catkin_ws/src/waterplus_map_tools/scripts
./install_for_noetic.sh
```

注意, 编译后, 需要让 .baserh 写东西 (见视频)

- roscore
- roslaunch wpr_simulation wpb_simple.launch

如果启动闪一下橙色窗口, 并且报错. 请检查 VcXsrv 连接的时候是否配置正确!!!