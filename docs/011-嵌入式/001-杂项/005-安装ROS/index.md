# 安装ROS in ubt20.04

- https://www.bilibili.com/video/BV1aP41137k9/

# 给虚拟机配置代理

默认算是在局域网的, 所以在代理上面允许局域网使用代理, 然后配置即可

命令行使用代理:

```sh
export http_proxy=socks5://192.168.213.1:2333
export https_proxy=socks5://192.168.213.1:2333
```

# 可能需要的

- Hyper-V安装的Ubuntu虚拟机提示“磁盘空间不足”的硬盘扩容方案 
    - https://www.cnblogs.com/kendoziyu/p/16276417.html

# 忽略以下内容

- https://blog.vogelcs.com/2024/05/21/Windows%E4%B8%8B%E7%9B%B4%E9%80%9AGPU%E8%AE%BE%E5%A4%87%E5%88%B0Hyper-V%E7%9A%84Ubuntu%E8%99%9A%E6%8B%9F%E6%9C%BA/

- 设置共享分区

```cpp
mount -o username=loli,password=llh282 //192.168.213.1/loli-all ~/loli-all
```

```CPP
#!/bin/bash -e
BRANCH=linux-msft-wsl-5.10.y

if [ "$EUID" -ne 0 ]; then
    echo "Swithing to root..."
    exec sudo $0 "$@"
fi

apt-get install -y git dkms

git clone -b $BRANCH --depth=1 https://github.com/microsoft/WSL2-Linux-Kernel
cd WSL2-Linux-Kernel
VERSION=$(git rev-parse --short HEAD)

cp -r drivers/hv/dxgkrnl /usr/src/dxgkrnl-$VERSION
mkdir -p /usr/src/dxgkrnl-$VERSION/inc/{uapi/misc,linux}
cp include/uapi/misc/d3dkmthk.h /usr/src/dxgkrnl-$VERSION/inc/uapi/misc/d3dkmthk.h
cp include/linux/hyperv.h /usr/src/dxgkrnl-$VERSION/inc/linux/hyperv_dxgkrnl.h
sed -i 's/\$(CONFIG_DXGKRNL)/m/' /usr/src/dxgkrnl-$VERSION/Makefile
sed -i 's#linux/hyperv.h#linux/hyperv_dxgkrnl.h#' /usr/src/dxgkrnl-$VERSION/dxgmodule.c
echo "EXTRA_CFLAGS=-I\$(PWD)/inc" >> /usr/src/dxgkrnl-$VERSION/Makefile

cat > /usr/src/dxgkrnl-$VERSION/dkms.conf <<EOF
PACKAGE_NAME="dxgkrnl"
PACKAGE_VERSION="$VERSION"
BUILT_MODULE_NAME="dxgkrnl"
DEST_MODULE_LOCATION="/kernel/drivers/hv/dxgkrnl/"
AUTOINSTALL="yes"
EOF

dkms add dxgkrnl/$VERSION
dkms build dxgkrnl/$VERSION
dkms install dxgkrnl/$VERSION
```

- 每次都要分配gpu
```C++
Set-VM -VMName ubt -GuestControlledCacheTypes $true -LowMemoryMappedIoSpace 1GB -HighMemoryMappedIoSpace 32GB
Add-VMGpuPartitionAdapter -VMName ubt
```

- 再次打开, 报错, 就先关闭: 再打开↑
```cpp
Remove-VMGpuPartitionAdapter -Vmname ubt
```

去除分区
```C++
Set-VMProcessor -VMName ubt -ExposeVirtualizationExtensions $false

Set-VMProcessor -VMName ubt -ExposeVirtualizationExtensions $true

Remove-VMGpuPartitionAdapter -Vmname ubt
```
