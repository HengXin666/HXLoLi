# Linux用户管理
linux系统是一个多用户多任务的操作系统，任何一个要使用系统资源的用户，都必须首先向系统管理员申请一个账号，然后以这个账号的身份进入系统。

## 添加用户
在root用户下，我们可以创建很多其它的用户，并且这些用户都会生成对应的目录，这些目录位于`/home/用户名`的目录下面。如果我们使用自己创建的用户登录，默认的情况下，用户所在的目录就是`/home/用户`目录所在的位置。

```Bash
useradd [name] # 创建用户
```

细节：

当用户创建成功后，会自动的创建和用户同名的目录。这个目录位于`/home`下面。

也可以通过 `useradd -d` 来指定目录新的用户名，给新的用户名指定目录

```Shell
[root@hxlinux home]# useradd -d /home/text gg
[root@hxlinux home]# ll
总用量 4
drwx------. 15 heng_xin heng_xin 4096 1月   4 01:48 heng_xin
drwx------.  3 hx       hx         78 1月   4 03:55 hx
drwx------.  3 gg       gg         78 1月   4 03:56 text
```

## 给用户添加密码

创建密码的命令： `passwd [用户名]`

注意: 

1. 输入密码是不会显示的, 连`*`也不会有，但是已经是接受输入了!
2. 提示 `无效的密码` != 失败, 只是密码不安全而已, 重新输入和刚刚一样的密码, 就`创建密码`成功了

```Shell
[root@hxlinux home]# passwd gg
更改用户 gg 的密码 。
新的 密码：
无效的密码： 密码少于 8 个字符
重新输入新的 密码：
passwd：所有的身份验证令牌已经成功更新。
```

## 删除用户
删除用户有两种情况，一种是删除用户，`保存用户对应的目录`。还有一种是删除用户，`连用户对应的目录也删除掉`。

*注意: 需要有root权限*

```Bash
userdel [用户名]    # 删除用户，保存用户对应的目录

userdel -r [用户名] # 删除用户，对应的用户目录也删除掉 (常用)
```

## 查询用户信息
### id

查询用户的详细信息 `id [用户名]`
```Shell
[root@hxlinux home]# id root
uid=0(root) gid=0(root) 组=0(root)
```

`uid` - **用户ID** | `gid` - **组ID**

## whoami

查询当前用户是谁
```Shell
[root@hxlinux home]# whoami
root
```

## who am i

查询当前用户的详细信息
```Shell
[root@hxlinux home]# who am i
root     pts/0        2024-01-04 01:48 (192.168.233.1)
```