# 3.2 Docker网络的基本命令
我们可以使用help命令去查看docker的常用命令有哪些:

## help
```bash
[root@localhost ~]# docker network help

Usage:  docker network COMMAND

Manage networks

Commands:
  connect     Connect a container to a network
  create      Create a network
  disconnect  Disconnect a container from a network
  inspect     Display detailed information on one or more networks
  ls          List networks
  prune       Remove all unused networks
  rm          Remove one or more networks

Run 'docker network COMMAND --help' for more information on a command.
```

## ls
```bash
[root@localhost ~]# docker network ls # 查看网络
NETWORK ID     NAME      DRIVER    SCOPE
f38a85f4e508   bridge    bridge    local
dc0aa33635b7   host      host      local
60b8afbeefb2   none      null      local
```

## inspect
```bash
[root@localhost ~]# docker network inspect bridge # 查看指定网络的详细信息
[
    {
        "Name": "bridge",
        "Id": "f38a85f4e508be1b3417deddb74a3e50219038df04986d5380a17deb817b62de",
        "Created": "2024-04-25T15:41:36.408203823+08:00",
        "Scope": "local",
        "Driver": "bridge",
        "EnableIPv6": false,
        "IPAM": {
            "Driver": "default",
            "Options": null,
            "Config": [
                {
                    "Subnet": "172.17.0.0/16", # 子网
                    "Gateway": "172.17.0.1" # 网关
                }
            ]
        },
        "Internal": false,
        "Attachable": false,
        "Ingress": false,
        "ConfigFrom": {
            "Network": ""
        },
        "ConfigOnly": false,
        "Containers": {},
        "Options": {
            "com.docker.network.bridge.default_bridge": "true",
            "com.docker.network.bridge.enable_icc": "true",
            "com.docker.network.bridge.enable_ip_masquerade": "true",
            "com.docker.network.bridge.host_binding_ipv4": "0.0.0.0",
            "com.docker.network.bridge.name": "docker0", # 网桥
            "com.docker.network.driver.mtu": "1500"
        },
        "Labels": {}
    }
]
```

## create 与 rm
```bash
[root@localhost ~]# docker network create network_abc # 创建一个网络
03dbbf140f32c887e7996a44be605444c0b0a353f74dec31c5584848438b5946
[root@localhost ~]# docker network ls
NETWORK ID     NAME          DRIVER    SCOPE
f38a85f4e508   bridge        bridge    local
dc0aa33635b7   host          host      local
03dbbf140f32   network_abc   bridge    local
60b8afbeefb2   none          null      local
[root@localhost ~]# docker network rm network_abc # 删除指定的网络
network_abc
[root@localhost ~]# docker network ls
NETWORK ID     NAME      DRIVER    SCOPE
f38a85f4e508   bridge    bridge    local
dc0aa33635b7   host      host      local
60b8afbeefb2   none      null      local
```
