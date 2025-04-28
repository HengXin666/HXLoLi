# WSL2
## 安装WSL2 Linux子系统

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

## 远程连接桌面

- 通过 VcXsrv 在 WSL2 上使用图形化界面（xfce4）

---
注: 您可以先只安装 VcXsrv, 然后看看: WSL2 使用 KDE (这个界面我喜欢) | 即 请跳到 `配置GPU` 处, 继续往下

---

-
- ~~https://www.cnblogs.com/blauendonau/p/14166062.html~~

~~连接桌面:~~

```C++
startxfce4
```

## 配置GPU

使用
```C++
nvidia-smi
```
查看是否有驱动

若没有, 则使用
```C++  
sudo apt-cache search nvidia-*
```  
能查看到当前可以使用的所有版本 (实际上, `nvidia-smi`也会列出需要安装的版本, 选一个新的, 不是服务端的驱动安装即可)

如:

```sh
loli@HengXin-ROG-PRIME:~$ nvidia-smi

Command 'nvidia-smi' not found, but can be installed with:

sudo apt install nvidia-utils-435         # version 435.21-0ubuntu7, or
sudo apt install nvidia-utils-440         # version 440.82+really.440.64-0ubuntu6
sudo apt install nvidia-340               # version 340.108-0ubuntu5.20.04.2
sudo apt install nvidia-utils-390         # version 390.157-0ubuntu0.20.04.1
sudo apt install nvidia-utils-450-server  # version 450.248.02-0ubuntu0.20.04.1
sudo apt install nvidia-utils-470         # version 470.256.02-0ubuntu0.20.04.1
sudo apt install nvidia-utils-470-server  # version 470.256.02-0ubuntu0.20.04.1
sudo apt install nvidia-utils-535         # version 535.183.01-0ubuntu0.20.04.1
sudo apt install nvidia-utils-535-server  # version 535.183.06-0ubuntu0.20.04.1
sudo apt install nvidia-utils-550-server  # version 550.90.07-0ubuntu0.20.04.2
sudo apt install nvidia-utils-418-server  # version 418.226.00-0ubuntu0.20.04.2

loli@HengXin-ROG-PRIME:~$ sudo apt install nvidia-utils-535
# ... 略
loli@HengXin-ROG-PRIME:~$ nvidia-smi
Tue Sep 17 11:37:31 2024
+---------------------------------------------------------------------------------------+
| NVIDIA-SMI 535.183.01             Driver Version: 560.94       CUDA Version: 12.6     |
|-----------------------------------------+----------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |         Memory-Usage | GPU-Util  Compute M. |
|                                         |                      |               MIG M. |
|=========================================+======================+======================|
|   0  NVIDIA GeForce RTX 4080 ...    On  | 00000000:01:00.0 Off |                  N/A |
| N/A   41C    P8               1W / 175W |     37MiB / 12282MiB |      0%      Default |
|                                         |                      |                  N/A |
+-----------------------------------------+----------------------+----------------------+

+---------------------------------------------------------------------------------------+
| Processes:                                                                            |
|  GPU   GI   CI        PID   Type   Process name                            GPU Memory |
|        ID   ID                                                             Usage      |
|=======================================================================================|
|  No running processes found                                                           |
+---------------------------------------------------------------------------------------+
```

安装需要的版本, 使用:
```sh
sudo apt-get install nvidia-版本号 
```

## 美化: 使用KDE桌面

- https://gist.github.com/camullen/0c41d989ac2ad7a89e75eb3be0f8fb16

[在 WSL2 上安装 KDE ·GitHub的](https://gist.github.com/camullen/0c41d989ac2ad7a89e75eb3be0f8fb16#file-installation-md)

### 在 WSL2 上安装 KDE

[](https://gist.github.com/camullen/0c41d989ac2ad7a89e75eb3be0f8fb16#installing-kde-on-wsl2)

灵感： [https://www.most-useful.com/kde-plasma-on-wsl.html](https://www.most-useful.com/kde-plasma-on-wsl.html)

#### 设置

[](https://gist.github.com/camullen/0c41d989ac2ad7a89e75eb3be0f8fb16#setup)

1.  更新 WSL
    -   在 Windows 命令提示符下运行： `wsl --update`
2.  将 systemd 添加到 ubuntu
    -   在 ubuntu 提示符下运行： `sudo nano /etc/wsl.conf`
    -   将以下内容添加到文件中：
        
            [boot]
            systemd=true
            
        
    -   按 ctrl + o，然后按 ctrl + x 保存并退出
    -   通过在 Windows 命令提示符中运行以下命令来关闭 WSL： `wsl --shutdown`
    -   通过打开 Ubuntu 的新终端会话来重启 WSL

#### 安装 KDE

```sh
sudo apt update

sudo apt install kubuntu-desktop
```

在设置过程中选择 SDDM 作为显示管理器

`sudo apt install lightdm`

在安装过程中，使用箭头键选择 lightdm 作为默认显示管理器

编辑 lightdm 配置，如下所示:

`sudo nano /etc/lightdm/lightdm.conf`

添加以下文本

```sh
[Seat:*]
user-session=plasma

[LightDM]
start-default-seat=false

[XDMCPServer]
enabled=true
port=177
```

按 ctrl + o，然后按 ctrl + x 保存并退出

#### 设置 x 服务器

[](https://gist.github.com/camullen/0c41d989ac2ad7a89e75eb3be0f8fb16#setting-up-x-server)

下载并安装此软件 Windows 版：

[https://sourceforge.net/projects/vcxsrv/](https://sourceforge.net/projects/vcxsrv/)

#### 启动 Desktop

[](https://gist.github.com/camullen/0c41d989ac2ad7a89e75eb3be0f8fb16#launch-desktop)

-   在 Windows 中打开 **XLaunch** 程序
-   选择 `One window without titlebar` 选项，然后单击 `Next`
-   选择 `Open session via XDMCP` ，然后单击 Next
-   在 ubuntu 终端中，您需要获取 wsl2 实例的 IP 地址：
-   输入`ip addr`即可!
-   IP 地址将列在 `inet` item 的 intent 和通常看起来像 172.xxx.xxx.xxx
-   复制上面的 IP 地址并粘贴到旁边的文本框中 `Connect to host` ，然后单击 `next`
> -   修改默认设置以取消选中 `Native opengl` 并选择 `Disable access control` (就是中间不选, 其他全选)
-   点击 `Next`
-   点击 `Save configuration` 并将文件保存到桌面，并将文件命名为您想要的任何名称
-   **至关重要：当您首次运行 VcXSrv（或 XLaunch）时，您会看到一个 Windows 防火墙弹出窗口。您需要允许它接受 PRIVATE 和 PUBLIC 网络上的连接。如果不允许 PUBLIC 网络，则不会获得连接，因为 WSL 虚拟网络被视为 PUBLIC**
-   点击 `Finish`

#### 加速 KDE

[](https://gist.github.com/camullen/0c41d989ac2ad7a89e75eb3be0f8fb16#speeding-up-kde)

#### 修复渲染

[](https://gist.github.com/camullen/0c41d989ac2ad7a89e75eb3be0f8fb16#fixing-rendering)-   在 KDE 桌面上，单击开始菜单，转到 `Computer` 然后点击 `System Settings`
-   下面 `Hardware` 在左侧面板中，单击 `Display and Monitor`
-   选择 `Compositor` 从左侧面板
-   更改 `rendering backend` 自 `XRender`
-   检查 `Enable compositor on startup`
-   点击 `Apply`

## 可能需要的代理(自用)

```sh
export http_proxy=http://192.168.213.1:2334
export https_proxy=http://192.168.213.1:2334
```

- 网线
```C++
export http_proxy=http://10.61.96.4:2334
export https_proxy=http://10.61.96.4:2334

export all_proxy=http://10.61.96.4:2334
```

- 网卡

```C++
export http_proxy=http://10.61.95.171:2334
export https_proxy=http://10.61.95.171:2334
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
