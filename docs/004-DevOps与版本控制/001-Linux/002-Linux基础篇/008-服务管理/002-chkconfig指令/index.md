# Linux服务管理指令之chkconfig指令
- 通过`chkconfig`可以给服务的**各个运行级别**设置自启动/关闭。
- `chkconfig`指令管理的服务在 `/etc/init.d`查看。
- <span style="color:red">**注意**：在 $Centos 7.0$ 以后，很多服务使用`systemctl`管理。</span>

基本用法:

- 查看服务: `chkconfig --list`
- 设置服务在**指定级别**启动/关闭: `chkconfig --level [服务名] [on/off]`

示例:
```Shell
[root@hxlinux init.d]# chkconfig --list

注：该输出结果只显示 SysV 服务，并不包含
原生 systemd 服务。SysV 配置数据
可能被原生 systemd 配置覆盖。 

      要列出 systemd 服务，请执行 'systemctl list-unit-files'。
      查看在具体 target 启用的服务请执行
      'systemctl list-dependencies [target]'。

netconsole         0:关    1:关    2:关    3:关    4:关    5:关    6:关
network            0:关    1:关    2:开    3:开    4:开    5:开    6:关
vmware-tools       0:关    1:关    2:开    3:开    4:开    5:开    6:关
vmware-tools-thinprint    0:关    1:关    2:开    3:开    4:开    5:开    6:关
```

说明:

即, 服务`network`在 0 ~ 6 的[Linux运行级别](../../001-基本常识/009-Linux运行级别/index.md)下的自启动状态如下:

```Shell
[root@hxlinux init.d]# chkconfig --list | grep network

注：该输出结果只显示 SysV 服务，并不包含
原生 systemd 服务。SysV 配置数据
可能被原生 systemd 配置覆盖。 

      要列出 systemd 服务，请执行 'systemctl list-unit-files'。
      查看在具体 target 启用的服务请执行
      'systemctl list-dependencies [target]'。

network            0:关    1:关    2:开    3:开    4:开    5:开    6:关
```

案例: 对于network服务，进行各种操作，把network在3运行级别，关闭自启动。
```Shell
[root@hxlinux init.d]# chkconfig --level 3 network off
[root@hxlinux init.d]# chkconfig --list | grep network

注：该输出结果只显示 SysV 服务，并不包含
原生 systemd 服务。SysV 配置数据
可能被原生 systemd 配置覆盖。 

      要列出 systemd 服务，请执行 'systemctl list-unit-files'。
      查看在具体 target 启用的服务请执行
      'systemctl list-dependencies [target]'。

network            0:关    1:关    2:开    3:关    4:开    5:开    6:关
[root@hxlinux init.d]# chkconfig --level 3 network on
```

**注意：`chkconfig`重新设置服务自启动或者关闭，需要重启机器`reboot`生效。**