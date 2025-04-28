# VM Linux抽风 - 「找不到网络适配器」
2024年4月27日, 我的龙蜥8突然抽风 -- 我使用xshell发现链接超时了

于是我打开图形化界面进入root账号, `ip addr`发现`ens33`的ip消失了(一开始没有仔细看, 把别的ip当成是`ens33`的了, 还纳闷怎么ip变了, 但是 ping 也 不通, 浏览器也打不开, 去设置那看: wifi那里直接`找不到网络适配器`, 甚至wifi都消失了, 只能在搜索里面找到)

尝试了各种办法: 修改什么本机模式/桥接模式/初始化nat模式/修改windows下的VM的服务/配置网关为DNS 统统不行!

并且 VM 虚拟机详细信息-主 IP 地址: 网络信息不可用 (可能是需要开机才可以看到)

然后我就进入之前的win10镜像, 同样使用同一个nat模式, 但是wifi是OK的, 我百思不得其解, 因为昨天还好好的, 今天突然就抽风了!

找了好多教程, 直到: [Ubuntu上不了网：ifconfig查看只有lo,没有ens33问题解决参考方法](https://blog.csdn.net/qq_41969790/article/details/103222251)

并且使用GPT-3.5把命令转为龙蜥的:

```bash
sudo systemctl stop NetworkManager # 首先停止网络服务
sudo rm -rf /var/lib/NetworkManager/NetworkManager.state # 删除设备网卡状态管理文件
sudo systemctl start NetworkManager # 重新启动网络服务，这时网络状态将会重新加载刷新写入到文件
```

然后就OK了?!

我日, 一个下午啊! 你怎么就突然抽风了?! 艹! FUCK! 八嘎!