# Linux磁盘情况查询指令
## 查询整个磁盘占用情况
`df -h`: 查询整个磁盘占用情况.

```Shell
[root@hxlinux ~]# df -h
文件系统        容量  已用  可用 已用% 挂载点
devtmpfs        3.9G     0  3.9G    0% /dev
tmpfs           3.9G     0  3.9G    0% /dev/shm
tmpfs           3.9G  9.7M  3.9G    1% /run
tmpfs           3.9G     0  3.9G    0% /sys/fs/cgroup
/dev/sda3        51G  5.6G   46G   11% /
/dev/sdb1       991M  2.6M  922M    1% /hx_loli_home
/dev/sda1       3.0G  179M  2.9G    6% /boot
vmhgfs-fuse     884G   63G  821G    8% /mnt/hgfs
tmpfs           782M   12K  782M    1% /run/user/42
tmpfs           782M     0  782M    0% /run/user/0
```

## 查询指定目录的磁盘占用情况

```Bash
du -h [目录]
```
|参数|含义|
|:--:|:--:|
|-s|指定目录大小汇总|
|-h|带计量单位|
|-a|含文件|
|--max-depth=1|子目录深度(不指定会遍历全部子目录)|
|-c|列出明细的同时，增加汇总值|


```Shell
[root@hxlinux ~]# du -h --max-depth=1 /opt # 汇总opt目录下面的磁盘占用情况
0    /opt/rh
157M    /opt/vmware-tools-distrib
208M    /opt
```

汇总和含文件查询:
```Shell
[root@hxlinux ~]# du -hac --max-depth=1 /opt/
0    /opt/rh
51M    /opt/VMwareTools-10.0.12-4448491.tar.gz
157M    /opt/vmware-tools-distrib
208M    /opt/
208M    总用量
```