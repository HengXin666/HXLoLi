# Linux rpm指令
## 介绍

rpm是互联网下载包和打包和安装工具，他包含在某些linux分版中，他具有生产.rpm扩展名的文件，RPM是redhat package manage（软件包管理工具的缩写），类似于**setup.exe**。

## 查询所有安装的rpm列表

```Shell
[root@hxlinux ~]# rpm -qa | more # 分页展示
java-1.8.0-openjdk-headless-1.8.0.222.b03-1.el7.x86_64
perl-podlators-2.5.1-3.el7.noarch
clutter-1.26.2-2.el7.x86_64
python-ldap-2.4.15-2.el7.x86_64
cifs-utils-6.2-10.el7.x86_64
telepathy-filesystem-0.0.2-6.el7.noarch
mesa-libxatracker-18.3.4-5.el7.x86_64
perl-PathTools-3.40-5.el7.x86_64
libgweather-3.28.2-2.el7.x86_64
```

## 查询当前系统中是否安装了指定的软件

```Shell
[root@hxlinux ~]# rpm -qa | grep firefox # 查询是否安装火狐浏览器
firefox-60.8.0-1.el7.centos.x86_64
```

### 一个rpm包的名称命名规则

- `firefox`: rpm包名
- `60.8.0-1.el7`: rpm包的版本号
- `centos`: rpm包适用于的操作系统
- `x86_64`: 适用于64位的操作系统。(如果是`i686`或者`i386`说明适用于32位操作系统，`noarch`表示通用。)

## 查看软件包是否安装

```Shell
[root@hxlinux ~]# rpm -q firefox
firefox-60.8.0-1.el7.centos.x86_64
[root@hxlinux ~]# rpm -q mysql
未安装软件包 mysql
```

## 查询软件安装的详细信息

```Bash
rpm -qi [软件包名称]
```

```Shell
[root@hxlinux ~]# rpm -qi firefox
Name        : firefox
Version     : 60.8.0
Release     : 1.el7.centos
Architecture: x86_64
Install Date: 2023年12月21日 星期四 22时37分52秒
Group       : Unspecified
Size        : 218777805
License     : MPLv1.1 or GPLv2+ or LGPLv2+
Signature   : RSA/SHA256, 2019年07月12日 星期五 23时01分23秒, Key ID 24c6a8a7f4a80eb5
Source RPM  : firefox-60.8.0-1.el7.centos.src.rpm
Build Date  : 2019年07月12日 星期五 02时04分42秒
Build Host  : x86-01.bsys.centos.org
Relocations : (not relocatable)
Packager    : CentOS BuildSystem <http://bugs.centos.org>
Vendor      : CentOS
URL         : https://www.mozilla.org/firefox/
Summary     : Mozilla Firefox Web browser
Description :
Mozilla Firefox is an open-source web browser, designed for standards
compliance, performance and portability.
```

## 查看rpm包安装之后的文件

```Bash
rpm -ql [软件包名称]
```

示例:
```Shell
[root@hxlinux ~]# rpm -ql firefox
/etc/firefox
/etc/firefox/pref
/usr/bin/firefox
/usr/lib64/firefox
/usr/lib64/firefox/LICENSE
/usr/lib64/firefox/application.ini
/usr/lib64/firefox/browser/blocklist.xml
/usr/lib64/firefox/browser/chrome
/usr/lib64/firefox/browser/chrome.manifest
/usr/lib64/firefox/browser/chrome/icons
/usr/lib64/firefox/browser/chrome/icons/default
/usr/lib64/firefox/browser/chrome/icons/default/default128.png
```

## 查看指定的文件所属的rpm包

```Bash
rpm -qf [软件包名称]
```

```Shell
[root@hxlinux etc]# rpm -qf ./my.cnf # 查询/etc目录下面的my.cnf文件所属的rpm包
mariadb-libs-5.5.64-1.el7.x86_64
```

## 卸载已安装的rpm包
如果某一个rpm包我们不想要了，我们也可以卸载。

```Shell
[root@hxlinux opt]# rpm -e firefox # 删除firefox
```

### 强制卸载
注意：如果其它的软件包**依赖**于要删除的软件包，卸载时则会产生错误。此时如果我们想强制删除，可以添加另外的参数`--nodeps`。
```Shell
[root@hxlinux opt]# rpm -e --nodeps firefox # 删除firefox
```

## 安装rpm包

基本语法: `rpm -ivh RPM包全路径名称`

几个参数:
- i=install 安装
- v=verbose 提示
- h=hash 进度条

### 案例: 使用rpm命令安装firefox

```Shell
[root@hxlinux ~]# rpm -e firefox # 删除firefox
[root@hxlinux ~]# rpm -q firefox # 查询firefox是否删除成功
package firefox is not installed
```

接下来我们需要去下载rpm包，其实linux操作系统内置了firefox的安装包，我们
- 打开光驱
- --> 找到Packages目录
- --> 找到firefox的rpm包
- --> 将其拷贝至/opt目录下面

最后我们安装rpm包:

```Shell
[root@hxlinux ~]# rpm -ivh /opt/firefox-60.8.0-1.el7.centos.x86_64.rpm
```

最后我们发现firefox又安装成功了.