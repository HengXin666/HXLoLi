# shell中的循环
## for 循环
### 基本语法一: `for-in`
```bash
for i in 值1, 值2, ..., 值n
do
    满足循环条件需要执行的代码
done
```

示例: 循环打印命令行输入的参数

```bash
#!\bin\bash
cot=1
for i in $@
do
    echo "输入的第 $cot 个参数是: $i"
    cot=$[$cot + 1]
done
```

### 基本语法二: `for((;;))`

```bash
for(( 初始值; 循环控制条件; 变量变化)) # (()) 里面的空格可有可无
do
    满足循环条件需要执行的代码
done
```

示例:

```bash
#!\bin\bash
for ((i = 0; i < 100; ++i))
do
    echo $i
done
```

## while 循环
基本语法:
```bash
while [ 表达式 ] # 注意两端有空格
do
    代码
done
```

示例:

```bash
#!\bin\bash
# 求 1 + 2 + 3 + ... + 100
SUM=0
i=0
while [ $i -le 100 ]
do
    SUM=$[$SUM + $i]
    i=$[$i + 1]
done
echo "sum = $SUM"
```
