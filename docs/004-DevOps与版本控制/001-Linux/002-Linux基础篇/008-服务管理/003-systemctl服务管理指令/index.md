# Linux服务管理之systemctl指令
- `systemctl`指令管理的服务在`/usr/lib/systemd/system`中查看。
- 服务启动/停止/重启/重载/查看状态： `systemctl [start | stop | restart | status] [服务名]`

## 查看所有服务的自启动状态

`systemctl list-unit-files`

```Shell
[root@hxlinux init.d]# systemctl list-unit-files
UNIT FILE                                     STATE   
proc-sys-fs-binfmt_misc.automount             static  
dev-hugepages.mount                           static  
dev-mqueue.mount                              static  
proc-fs-nfsd.mount                            static  
proc-sys-fs-binfmt_misc.mount                 static  
sys-fs-fuse-connections.mount                 static  
sys-kernel-config.mount                       static  
sys-kernel-debug.mount                        static  
tmp.mount                                     disabled
var-lib-nfs-rpc_pipefs.mount                  static  
brandbot.path                                 disabled
cups.path                                     enabled 
systemd-ask-password-console.path             static  
systemd-ask-password-plymouth.path            static  
systemd-ask-password-wall.path                static  
session-5.scope                               static  
session-9.scope                               static  
abrt-ccpp.service                             enabled 
```

服务的状态如下：
- `masked`: 此服务禁止自启动
- `static`: 该服务无法自启动，只能作为其他文件的依赖
- `enabled`: 已配置为自启动
- `disabled`: 未配置为自启动

### 查看某一服务是否自启动

`systemctl is-enabled [服务名称]`

```Shell
[root@hxlinux init.d]# systemctl is-enabled firewalld.service # 需要知道该服务的名称
enabled
```

### 设置服务自启动 (服务运行级别 3、5)

```Shell
[root@hxlinux init.d]# systemctl enable firewalld.service
```

### 设置服务禁用自启动 (服务运行级别 3、5)

```Shell
[root@hxlinux init.d]# systemctl disable firewalld.service
```