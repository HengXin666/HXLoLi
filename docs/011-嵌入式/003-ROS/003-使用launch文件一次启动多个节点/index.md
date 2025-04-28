# 003.使用launch文件一次启动多个节点

我们可以通过`.launch`文件(实际上就是`.xml`格式的配置文件)

来实现一次启动多个节点

需要的操作如下

1. 在包目录下新建`launch`文件夹, 然后再在这个文件夹里面创建`xxx.launch`配置文件(名称随意)

```sh
ssr_pkg
├── CMakeLists.txt
├── include
│   └── ssr_pkg
├── launch
│   └── run_all.launch # .launch 文件
├── package.xml
└── src
    └── chao_node.cpp
```

2. 写入以下内容

```xml
<launch>
    <node pkg="ssr_pkg" type="chao_node" name="chao_node" output="screen" launch-prefix="konsole -e"/>
    <node pkg="ma_pkg" type="ma_node" name="ma_node" output="screen" launch-prefix="konsole -e"/>
</launch>
```

意思如下,

- `.launch`文件的配置需要被`<launch>`包裹

- `node`表示需要启动的结点

- `pkg`是包名, `type`是结点, `name`是名字

- `output="screen"`: 表示显示到屏蔽上

- `launch-prefix="konsole -e"`: 表示使用新建一个`konsole`控制台作为这个节点的输出

3. 运行`.launch`

```sh
roslaunch <包> <xxx.launch>
```

注: 

- 不需要`roscore`了, roslaunch 会自动运行`roscore`.

- <包>: 即`xxx.launch`放在哪个包下面了

如:

```sh
roslaunch ssr_pkg run_all.launch
```
