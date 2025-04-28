## 前后端分离
### 4 数据导出
假如我们现在想要实现一个数据导出功能，要求支持当前页面数据导出和全部数据导出，说说你的实现思路（实现步骤、前后端需要怎么配合以及需要考量会出现的额外问题）。

1. 实现思路
   - 首先【当前页面数据导出】可以直接使用本地的数据(因为已经在前端展示出来了吧)，而不需要再耗费流量到服务器的数据库中查找; 故直接打包当前界面的信息然后导出即可~~/或者如果没有展示, 那么就去数据库中查找, 然后分页好当前界面后打包返回~~
   - 其次【全部数据导出】就需要到远程服务器数据库中查找属于这个用户的全部数据，(按照某种顺序)并且打包好, 然后再发给用户
   - 步骤
       - 前端把界面做好, 比如明确导出当前界面还是导出全部, 然后发送对应的请求给后端
       - 后端根据请求, 查询对应内容, 然后打包好返回给前端

3. 前后端需要怎么配合
    - 前端发出请求, 然后说明请求的类型, 比如是要界面还是导出打包全部
    - 后端要处理请求, 并且根据请求的类型, 返回不同的结果

4. 需要考量会出现的额外问题
    - 网络问题, 可能用户会掉线,
    - 性能问题, 可能需要设计数据库/优化查询
    - 隐私安全, 要保护好用户的安全, 做好权限检查
    - 反爬/恶意请求

### 5 数据导入
假如我们现在想要实现一个数据导入功能，说说你的实现思路（实现步骤、前后端需要怎么配合以及需要考量会出现的额外问题）。

- 实现思路
    - 首先前端在本地先检测数据是否合法(大小限制/格式限制等), 然后post请求发送到后端, 后端再解析数据, 再次检测是否合法, 然后再导入到数据库, 返回成功或失败原因给前端反馈给用户 (如果是做网盘/上传图片/视频, 那么可能还要看看是否有重复的, 我们复用它(享元模式awa))
    - 可能出现的问题
        - 成本, 我们可以存放这么多吗?
        - 性能, 如果很多用户同上发送海量信息怎么办?
        - 用户恶意上传


## 操作系统操作
### 6 Docker的使用
#### 安装Docker需要的步骤
1. 安装gcc环境

```bash
yum -y install gcc

yum -y install gcc-c++
```

2. 安装需要的软件包

```bash
yum install -y yum-utils
```

3. 设置镜像仓库

```bash
yum-config-manager --add-repo http://mirrors.aliyun.com/docker-ce/linux/centos/docker-ce.repo
```

4. 更新yum软件包索引

```bash
yum makecache fast

yum makecache # 龙蜥使用这个
```

5. 安装Docker

```bash
yum -y install docker-ce docker-ce-cli containerd.io

# 如果出现问题, 可以使用 --allowerasing 来移除冲突的软件包
yum -y install docker-ce docker-ce-cli containerd.io --allowerasing
```

6. 启动Docker

```bash
systemctl start docker
```

#### 说说你是怎样使用Docker的，对应使用步骤需要列举出对应的命令行。

一般是直接拉取+部署(docker run)

可以运行镜像变成容器实例(如果镜像不存在会在云端拉取)

```bash
docker run 镜像名称:版本
```

并且可以添加参数 (`-d`) 是后台运行, `docker run -it 镜像名称:版本 /bin/bash` 可以通过控制台的形式进入容器内部, `-p`进行端口映射, `-v`挂载容器卷 等

以及 `docker ps` 查看当前运行的容器 / `docker images` 列出本机有的镜像

`docker log` 查看容器日志

`docker exec` 进入容器内部

ctrl + p + q 不停止日期的退出

当然还可以自己制作容器...

我真正应用docker实际上是使用docker-compose(我就只有部署的份(https://blog.hxloli.com/blog/#/home 就是我使用它部署的个人博客)), 它可以写.yaml, 并且自定义网络模式来可以通过容器名称进行网络传输...

## 7 Linux操作
### 如果要在Linux服务器上备份某些重要文件，请编写一个备份脚本


```bash
#!/bin/bash

# 定义备份源目录和目标备份目录
backup_source="/path/to/source"
backup_dest="/path/to/backup"

# 创建备份目录（如果不存在）
mkdir -p $backup_dest

# 定义备份文件名，加上当前日期时间作为时间戳
backup_file="backup_$(date +'%Y%m%d%H%M%S').tar.gz"

# 执行备份操作
tar -czf "$backup_dest/$backup_file" $backup_source

# 检查备份是否成功
if [ $? -eq 0 ]
then
    echo "备份成功: $backup_dest/$backup_file"
else
    echo "备份失败"
fi
```

然后给上权限`chmod +x 备份脚本.sh`, 然后可以写在`crontab -e`定时任务里面`*/1 * * * * 路径/备份脚本.sh`(1天备份一次)

## 编码调试能力
### 8 调试步骤
说说你在调试解决问题的步骤什么？

1. 发现问题(常态性突发): 你运行的时候发现和设想的不一样, 或者出现bug, 等等
2. 定位问题: 按照理解, 从最近写过的代码逻辑中, 寻找可能会出现问题/疑问/不确定的地方, 打下断点(如果没有头绪就从绝对没有问题的地方打断点, 然后一个函数/方法这样跳, 然后看看是哪一步开始出现问题)
3. 解决问题: 看看是不是代码逻辑/bug/需要特判/越界...等问题，然后就是断点调试看看变量的值哪里开始出问题等等 (一般都可以解决, 最多就是时间问题)
4. 如果解决不了问题就解决问题本身! (直接重写这个出问题的地方(不改了awa..))

## 10 数据库题组一
### （1） 第一题

```sql
select
    count(id) as male_num,
    round(avg(total_time)) as avg_total_time
from
    players
where
    gender = "male";
```

### （2） 第二题

```sql
select
    a.emp_no,
    a.name,
    b.salary
from
    employees as a
join
    salaries as b
on
    a.emp_no = b.emp_no
where
    b.emp_no < (select max(salary) from salaries);
```

## 11 数据库题组二
### （1）第一题

```sql
select
    a.user_id,
    a.name,
    a.grade_sum as grade_num
from
(
    select -- 得到所有的分数
        a.user_id,
        a.name
        sum(if(b.type = "+", b.grade_num, -b.grade_num)) as grade_sum
    from
        user as a
    join
        grade_info as b
    on
        a.user_id = b.user_id
    group by
        a.user_id
) as a
where
    a.grade_sum = ( -- 单词查询所有分数的最大值
    select
        max(sum(if(b.type = "+", b.grade_num, -b.grade_num)))
    from
        grade_info as b
    group by
        b.user_id
)
order by
    a.user_id asc
```

### （2）第二题

```sql
select
    ROW_NUMBER() OVER (ORDER BY first_year_mon ASC, topic ASC) as id,
    topic,
    a.ym as first_year_mon,
    a.s as first_year_cnt,
    b.ym as second_year_mon,
    b.s as second_year_cnt
from
(select
    DATE_FORMAT(`date`, "%Y-%m") as ym,
    sum(num) as s
from
    visit_records
where
    YEAR(`date`) = 2025
group by
    topic, MONTH(`date`)) as a,
(select
    DATE_FORMAT(`date`, "%Y-%m") as ym,
    sum(num) as s
from
    visit_records
where
    YEAR(`date`) = 2026
group by
    topic, MONTH(`date`)) as b
where
    a.topic = b.topic and MONTH(a.ym) = MONTH(b.ym)
```

## 简答题
### 13 简答题1
面向对象编程是一种编程范式，它将程序中的数据和操作封装在对象中，通过对象之间的交互来实现程序的功能。面向对象编程的核心概念包括封装、继承和多态。

C++中可以通过`class`定义一个类, 使用`protected`权限修饰符来对外隐藏成员 以达到封装的目的 ,`class A : public B` 来继承一个类, 使用`virtual`关键字定义一个虚函数, 然后使用`基类 变量名 = new 子类`, 但可以实现动态绑定, 以实现多态

### 14 简答题2

```C++
#define MAX(arrName, size, type) [&]() -> type {type res = arrName[0];\
    for (int i = 1; i < size; ++i) { \
        if (res < arrName[i]) \
            res = arrName[i];\
    }\
    return res;\
}()
```

### 15 简答题3
stl共用的优点是支持泛型, 可以让用户自定义类型

我常用的stl库有
- vector
    - 它是一个动态数组, 优点: 支持随机访问，查找速度快, 动态增长(back_push())，方便管理和使用
    - 缺点: 因为是数组, 所以整体移动需要o(n)的时间复杂度, 并且可能增长到某个长度就需要整体拷贝到别的空间; 不适合频繁插入和删除操作

- set/map
    - 它的底层是一个红黑树, 可以实现一个 logn级别的时间复杂度来查找和删除元素, 并且自带排序, 甚至可以把它作为一个有序序列
    - 缺点: 某些时候红黑树还是满.../自定义类型可能需要自己实现排序函数, 就比较麻烦

- unordered_set/map
    - 底层是哈希表 几乎是o(1)的时间复杂度, 对于需要很多删除修改的操作十分适合, 访问神速
    - 缺点: 就不能像`set/map`那样作为有序序列通过`begin/rbegin`取最值了, 需要开辟比较大的空间, 而且不是所有空间都被利用到, 还有哈希冲突等问题

- priority_queue
    - 优先队列, 可以自定义底层使用上面实现一般是voctor实现的堆, 可以取最大/最小值, logn时间复杂度
    - 缺点: 写法繁琐(很长), ...好吧只是算法题用的, 不知道有什么特别的缺点

- tuple/pair
    - 非常方便! 这样对于基本数据类型, 可以不用实现上面的排序函数, 而自动按照k1,k2,k3这样进行默认的排序
    - 缺点: tuple 需要配合 tie() 或者 [] 结构化绑定使用才爽, 而环境需要c++17, 此外如果不熟练可能使用引用的时候会出现问题, 让人即熟悉又陌生