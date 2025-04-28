# 四、Docker-compose容器编排
## 4.2 Docker Compose使用的三个步骤
1. 编写 DockerFile 定义各个微服务应用并构建出对应的镜像文件

2. 使用`docker-compose.yml`定义一个完整业务单元，安排好整体应用中的各个容器服务。

3. 最后，执行`docker-compose up`命令 来启动并运行整个应用程序，完成一键部署上线

<div style="margin-top: 80px;">

---
</div>

## 4.3 Docker Compose的常用基本命令

```bash
docker-compose -h # 查看帮助
docker-compose up # 启动所有docker-compose服务
docker-compose up -d # 启动所有docker-compose服务并后台运行
docker-compose down # 停止并删除容器、网络、卷、镜像。
docker-compose exec yml里面的服务id # 进入容器实例内部
docker-compose exec docker-compose.yml文件中写的服务id /bin/bash
docker-compose ps # 展示当前docker-compose编排过的运行的所有容器
docker-compose top # 展示当前docker-compose编排过的容器进程
docker-compose logs yml里面的服务id # 查看容器输出日志
docker-compose config # 检查配置
docker-compose config -q # 检查配置，有问题才有输出

docker-compose restart # 重启服务
docker-compose start   # 启动服务
docker-compose stop    # 停止服务
```

## 4.4 Docker Compose的`.yaml`文件编写
内容比较多, 但是实际上都是学过的内容, 可以先学习一下`.yaml`([菜鸟教程 YAML 入门教程](https://www.runoob.com/w3cnote/yaml-intro.html)), 然后我们直接上实例!

这个是本博客/笔记的微服务的`Docker Compose`的`.yaml`实例, 我加了点注释, 看懂即可: 指令的详细介绍: [菜鸟教程 Docker Compose](https://www.runoob.com/docker/docker-compose.html)

```yaml
# 指定版本, 目前最新为 3.9 可向下兼容
version: "3.9"

# 创建一个自定义网络模式, 并且命名为 blossomnet
networks:
  blossomnet:
    # 自定义网络模式基于 bridge 模式
    driver:
      bridge

# 服务
services:
  # 微服务容器
  blossom:
    # 基于的镜像
    image: jasminexzzz/blossom:latest

    # (运行后)该容器的名称(类似于 --name)
    container_name: blossom-backend

    # 磁盘挂载
    volumes:
      # 【需修改】 
      # 将冒号(:)前的部分改成你运行 docker 的设备的某个路径，不要修改冒号后面的内容。  
      # 如果是windows环境，可以使用/c/home/bl/img/来指定磁盘 
      # 该配置十分重要，所有的图片和备份文件都在这个路径下 
      - /d/blossom/bl/:/home/bl/ 
      # 添加到该位置
      - /d/config.js:/application/BOOT-INF/classes/static/blog/config.js

    # 环境变量
    environment:
      SPRING_DATASOURCE_URL: jdbc:mysql://blmysql:3306/blossom?useUnicode=true&characterEncoding=utf-8&allowPublicKeyRetrieval=true&allowMultiQueries=true&useSSL=false&&serverTimezone=GMT%2B8
      SPRING_DATASOURCE_USERNAME: root
      # 【可选修改】配置数据库密码，这个改了下方的黄色部分也要修改 
      SPRING_DATASOURCE_PASSWORD: 密码 
      # 【需修改】配置图片上传后对应生成的访问 URL，需要以/pic/结尾。注意，该访问域名(IP:端口)需要与访问后台的域名(IP:端口)相同 
      PROJECT_IAAS_BLOS_DOMAIN: http://127.0.0.1:9999/pic/ 
      PROJECT_IAAS_BLOS_DEFAULT-PATH: /home/bl/img/

    # 端口映射
    ports:
      - "9999:9999"

    # 该容器使用的网络模式
    networks:
      - blossomnet

    # 用于检测 docker 服务是否健康运行
    healthcheck:
      test: ["CMD", "curl", "-f", "http://127.0.0.1:9999/sys/alive"] # 设置检测程序
      interval: 30s # 设置检测间隔
      timeout: 10s # 设置检测超时时间
      retries: 3 # 设置重试次数
      start_period: 5s # 启动后，多少秒开始启动检测程序

    # 重启策略
    # no：是默认的重启策略，在任何情况下都不会重启容器。
    # always：容器总是重新启动
    # on-failure：在容器非正常退出时（退出状态非0），才会重启容器。
    # unless-stopped：在容器退出时总是重启容器，但是不考虑在Docker守护进程启动时就已经停止了的容器
    # 注：swarm 集群模式，请改用 restart_policy。
    restart: always

    # 设置依赖关系
    depends_on:
      blmysql: # 必须要 blmysql容器 启动 后, 才启动 blossom容器
        condition: service_healthy
  
  # 数据库容器
  blmysql:
    # 基于的镜像与名称
    image: mysql:8.0.31
    container_name: blossom-mysql

    # 重启策略
    restart: on-failure:3 # 指连续(Docker 守护进程监控的时间范围内)3次以非零状态退出后, 将不会尝试重启容器

    # 磁盘挂载
    volumes:
      # 【需修改】将冒号(:)前的部分改成你运行 docker 的设备的某个路径，不要修改冒号后面的内容。  
      - /d/blossom/Docker/mysql/data:/var/lib/mysql 
      - /d/blossom/Docker/mysql/log:/var/log/mysql 
      - /d/blossom/Docker/mysql/mysql-files/log:/var/lib/mysql-files
    
    # 环境变量
    environment:
      MYSQL_DATABASE: blossom
      # 【可选修改】这个改了上方的黄色部分也要修改。需要与 services.blossom.environment.SPRING_DATASOURCE_PASSWORD 相同 
      MYSQL_ROOT_PASSWORD: 密码
      LANG: C.UTF-8
      TZ: Asia/Shanghai
    
    # 端口映射
    ports:
      - "3306:3306"

    # 基于的网络模式
    networks:
      - blossomnet

    # 测试程序
    healthcheck:
      # 【可选修改】如果修改了上方的数据库密码「MYSQL_ROOT_PASSWORD」修改，下方的 -p 后的密码也要修改
      test: ["CMD", "mysqladmin", "-uroot", "-p密码", "ping", "-h", "127.0.0.1"]
      interval: 10s
      timeout: 3s
      retries: 12
```

使用:

```bash
# 可以使用 -f 指定 .yaml 文件名(路径), 而不是必需要写那个默认的名字
docker compose -f blossom-mysql8.yaml up -d
# 如果命令无效，可使用下方命令尝试
docker-compose -f blossom-mysql8.yaml up -d
```

> 具体编写 + 运行微服务的话, 同之前使用 DockerFile 一样, 需要你打包好一个镜像, 而容器之间的通信就是通过你规定的容器名称来代替ip 即可 (因为设置了自定义网络模式了)