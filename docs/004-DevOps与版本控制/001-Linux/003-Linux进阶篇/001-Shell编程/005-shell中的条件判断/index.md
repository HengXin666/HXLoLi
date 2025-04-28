# shell中的条件判断
## if判断
基本语法: [ condition ] 注意: `condition`<span style="color:red">前后要有空格</span>。

```bash
if [ condition ]
then
    满足if语句执行的代码
fi
```

## if - else判断

```bash
if [ condition ]
then
    满足if语句执行的代码
else
    不满足if语句执行的代码
fi
```


### 常用的判断条件
1. 字符串比较 =

2. 两个整数比较
    - -lt(小于)
    - -gt(大于)
    - -le(小于等于)
    - -ge(大于等于)
    - -nt(不等于)

3. 按照文件权限进行判断
    - -r 有读的权限
    - -w 有写的权限
    - -x 有可执行的权限

4. 按照文件的类型进行判断
    - -f 文件存在，并且是一个常规文件
    - -e 文件存在
    - -d 文件存在并且文件是一个目录

示例:

```bash
#!\bin\bash
if [ $1 -ge 60 ] 
then
    echo "$1"
    echo "合格"
else
    echo "不合格"
fi

# 判断文件是否存在
if [ -f /root/main.cpp ]
then
    echo "文件存在"
else
    echo "文件不存在"
fi
```

### 特殊写法

```bash
if [ abc ] # 表示条件为 true
if [ ]     # 表示条件为 false
```

## 多重if - elif

```bash
if [ 表达式 ]
then
    为真执行
elif [ 表达式 ]
then
    为真执行
...
else
    全部为假, 执行
fi
```

示例:

```bash
#!\bin\bash
if [ $1 -ge 80 ]
then
    echo "优秀"
elif [ $1 -ge 60 ] 
then
    echo "合格"
else
    echo "不合格"
fi
```

```bash
[root@localhost shellCode]# sh 06_if.sh  -1
不合格
[root@localhost shellCode]# sh 06_if.sh  100
优秀
[root@localhost shellCode]# sh 06_if.sh  67
合格
```

## case语句
基本语法:

```bash
case $变量名 in
    "值1")
        如果变量名等于值1，执行的代码;;
    "值2")
        如果变量名等于值2，执行的代码;;
    ... ...
    *)
        如果都不满足以上的条件，则执行此代码;;
esac
```

示例:

```bash
#!\bin\bash
case $1 in
    "Yes")
        echo "这个是选择了对的";;
    "No")
        echo "这个是选择错了";;
    *)
        echo "纳尼, 居然输入: $1"
        echo "不能输入这个!!!";;
esac
```

```bash
[root@localhost shellCode]# sh 07_case.sh 1
纳尼, 居然输入: 1
不能输入这个!!!
[root@localhost shellCode]# sh 07_case.sh yes
纳尼, 居然输入: yes
不能输入这个!!!
[root@localhost shellCode]# sh 07_case.sh No
这个是选择错了
[root@localhost shellCode]# sh 07_case.sh Yes
这个是选择了对的
```
