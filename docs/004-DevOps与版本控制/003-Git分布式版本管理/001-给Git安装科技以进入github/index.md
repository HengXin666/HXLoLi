# Git配置篇: 安装科技
## 设置代理
由于网络原因，有的时候使用https的方式克隆、获取、拉取、推送代码会出现连接失败的问题，此时可以通过设置代理服务器的方式来解决问题。

使用代理的时候需要配置你的git，找到你的`.gitconfig`配置文件，一般在当前用户的根目录.

C盘 >> 用户 >> 当前用户名 >> `.gitconfig`


```.gitconfig
[http "https://github.com“】
    proxy = http://127.0.0.1:2333 # 此处改为你的代理服务器
```

## 配置SSH

### time out 解决方法

修改: `~/.ssh`下面的`config`文件

```config
Host github.com
HostName ssh.github.com  # 这是最重要的部分
User git
Port 443
PreferredAuthentications publickey
IdentityFile ~/.ssh/id_rsa
```


[关于本地git通过ssh链接github时 time out问题的解决方法](https://blog.csdn.net/the__future/article/details/130038818)