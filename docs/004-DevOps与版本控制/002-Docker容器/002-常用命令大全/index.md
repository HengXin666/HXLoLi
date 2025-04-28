# 常用命令大全

```
#info|version
docker info       #显示docker的系统信息，包括镜像和容器的数量
docker version    #显示docker的版本信息。

#帮助命令
docker 命令 --help #帮助命令

#镜像命令
docker images #查看所有本地主机上的镜像 可以使用docker image ls代替
docker search #搜索镜像
docker pull #下载镜像 docker image pull
docker rmi #删除镜像 docker image rm

#容器命令
docker run 镜像id #新建容器并启动
docker ps 列出所有运行的容器 docker container list
docker rm 容器id #删除指定容器
    #删除所有容器
    docker rm -f $(docker ps -aq)       #删除所有的容器
    docker ps -a -q|xargs docker rm  #删除所有的容器

#启动和停止容器
docker start 容器id    #启动容器
docker restart 容器id    #重启容器
docker stop 容器id    #停止当前正在运行的容器
docker kill 容器id    #强制停止当前容器

#退出容器
exit         #容器直接退出
ctrl +P +Q  #容器不停止退出     ---注意：这个很有用的操作

#其他常用命令
docker run -d 镜像名  #后台启动命令
docker logs         #查看日志
docker top 容器id     #查看容器中进程信息ps
docker inspect 容器id  #查看镜像的元数据
docker exec         #进入当前容器后开启一个新的终端，可以在里面操作。（常用）
docker attach         # 进入容器正在执行的终端
docker cp 容器id:容器内路径  主机目的路径    #从容器内拷贝到主机上

```