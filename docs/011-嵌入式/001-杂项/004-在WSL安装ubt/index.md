# WSL2安装ubt

- 需要win11专业版: https://zhuanlan.zhihu.com/p/682273555
    - 或者: 家庭版用户安装Hyper-v https://www.bilibili.com/read/cv26880911


- 打开`hyper-v` (https://www.bilibili.com/video/BV1Jj411b7j4/)

- 安装 https://www.bilibili.com/video/BV1QG411e7pn

---


大功告成! (下面的不用看了!!!) | 不对! tm的 没有显卡直通!, 还需要wsl!

---

```sh
export http_proxy=socks5://192.168.213.1:2333
export https_proxy=socks5://192.168.213.1:2333
```

apt的:


```C++
sudo nano /etc/apt/apt.conf
```

// 成功了, 需要使用http来代理! (自用)
```C++
Acquire::http::Proxy "http://192.168.213.1:2334";
Acquire::https::Proxy "http://192.168.213.1:2334";
Acquire::ftp::Proxy "http://192.168.213.1:2334";
```


## 远程连接桌面

- 通过 VcXsrv 在 WSL2 上使用图形化界面（xfce4）
- https://www.cnblogs.com/blauendonau/p/14166062.html

## 备用方法 (这个我连接就闪退...)

打开Windows功能，打开“适用于Linux的Windows子系统”和“虚拟机平台”
 
打开终端，依次输入以下命令：

- 查看可以安装的版本
```sh
wsl --list --online
```

- 安装需要的版本
```sh
wsl --install -d <安装的版本全称>
```

以下在Ubuntu界面输入：
```sh
sudo apt-get update
```

回到终端输入：
```sh
wsl -l -v
```
```sh
wsl --update
```
以管理员身份打开终端：

```sh
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```

```sh
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```


在Ubuntu中输入:

```sh
lsb_release -a

sudo apt update && sudo apt -y upgrade
 
sudo apt install xrdp
 
sudo apt install -y xfce4

sudo apt install calc

calc
 
exit
 
sudo apt install -y xfce4-goodies

sudo vi /etc/xrdp/startwm.sh
```

在最后一行添加:

```sh
startxfce4
```

```sh
service xrdp status

sudo /etc/init.d/xrdp start
```

然后我们在win上面搜索`远程桌面连接`, 输入ubt的ip

```sh
ip a 
 
sudo apt-get install xfce4-terminal
```