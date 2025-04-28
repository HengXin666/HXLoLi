# 3.3 容器内部网络ip的产生规则
现在我们通过一个案例给大家演示容器内部网络的产生规则，首先我们启动两个ubuntu容器实例

```bash
# 启动后使用 ctrl + p + q 退出!
docker run -it --name u1 ubuntu bash
docker run -it --name u2 ubuntu bash
```

现在我们查看两个容器的详细信息:

```bash
[root@localhost ~]# docker inspect u1 | tail -n 21
            "Networks": {
                "bridge": {
                    "IPAMConfig": null,
                    "Links": null,
                    "Aliases": null,
                    "MacAddress": "02:42:ac:11:00:02",
                    "NetworkID": "f38a85f4e508be1b3417deddb74a3e50219038df04986d5380a17deb817b62de",
                    "EndpointID": "c18427dca3d01fdce10de46a025fd3dcfb12f1c54d1e2fd35fc97026630b63e3",
                    "Gateway": "172.17.0.1",
                    "IPAddress": "172.17.0.2", # 注意 ip 地址
                    "IPPrefixLen": 16,
                    "IPv6Gateway": "",
                    "GlobalIPv6Address": "",
                    "GlobalIPv6PrefixLen": 0,
                    "DriverOpts": null,
                    "DNSNames": null
                }
            }
        }
    }
]
[root@localhost ~]# docker inspect u2 | tail -n 21
            "Networks": {
                "bridge": {
                    "IPAMConfig": null,
                    "Links": null,
                    "Aliases": null,
                    "MacAddress": "02:42:ac:11:00:03",
                    "NetworkID": "f38a85f4e508be1b3417deddb74a3e50219038df04986d5380a17deb817b62de",
                    "EndpointID": "e131e72ab0b51b2f80cbf3971e7434f349c7f506ef883ab22e7760f2f7dd712f",
                    "Gateway": "172.17.0.1",
                    "IPAddress": "172.17.0.3", # 注意 ip 地址
                    "IPPrefixLen": 16,
                    "IPv6Gateway": "",
                    "GlobalIPv6Address": "",
                    "GlobalIPv6PrefixLen": 0,
                    "DriverOpts": null,
                    "DNSNames": null
                }
            }
        }
    }
]
```

现在我们关闭容器u2，启动容器u3，观察u3的网络ip地址:

```bash
[root@localhost ~]# docker run -it --name u3 ubuntu bash
root@db0e531b9373:/# [root@localhost ~]# docker inspect u3 | tail -n 21
            "Networks": {
                "bridge": {
                    "IPAMConfig": null,
                    "Links": null,
                    "Aliases": null,
                    "MacAddress": "02:42:ac:11:00:03",
                    "NetworkID": "f38a85f4e508be1b3417deddb74a3e50219038df04986d5380a17deb817b62de",
                    "EndpointID": "f8578d9f02e3d8657ee579a458592bd817fff27d77dc2746ca1fdb98faca2542",
                    "Gateway": "172.17.0.1",
                    "IPAddress": "172.17.0.3", # 发现它的ip还是 u2 的
                    "IPPrefixLen": 16,
                    "IPv6Gateway": "",
                    "GlobalIPv6Address": "",
                    "GlobalIPv6PrefixLen": 0,
                    "DriverOpts": null,
                    "DNSNames": null
                }
            }
        }
    }
]
```

我们发现容器u3的地址和之前容器u2的地址是一样的！

现在我们得出一个结论: 在默认的网络情况下(bridge),容器内部的ip地址是可能会发生变化的(比如一个容器突然挂了, 并且被另一个刚刚启动的容器给顶替)，这种情况十分危险，比如我们想通过172.17.0.3这个ip去访问u2.但是现在却访问到了u3。

所以我们有必要详细了解docker的网络模式，来对docker网络进行规划。