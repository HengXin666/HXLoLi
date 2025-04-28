# shell函数
## 系统函数
shell编程和其他编程一样，有系统函数，也可以自定义函数。系统函数我们主要介绍两个。

- `basename` 获取文件名

去掉文件完整路径的多级路径 (和后缀名），返回文件名 。

基本语法: `basename 文件的完整路径 [文件后缀]`

应用实例: 请返回`/home/shellCode/a.txt`的文件名(`a.txt`)

```bash
[root@localhost shellCode]# basename /home/shellCode/a.txt
a.txt
```

- `dirname` 返回完整路径最前面部分
 
```bash
[root@localhost shellCode]# dirname /home/shellCode/a.txt
/home/shellCode
```

## 自定义函数
需求: 计算两个输入参数的和

如果需要, 请学习: [菜鸟教程-Shell 函数](https://www.runoob.com/linux/linux-shell-func.html)

语法:

```bash
[ function ] funname [()] {
    action
    [return int]
}
```

注意: return 语句只能返回一个介于 0 到 255 之间的整数，而两个输入数字的和可能超过这个范围。

```bash
#!\bin\bash
function getSum() {
    echo "执行了函数"
    return $[$1 + $2]
}

read -p "输入a: " a
read -p "输入b: " b

getSum $a $b
res=$? # 获取上一个命令的执行结果(即 函数的返回值)
       # 如果函数没有写return, 那么返回值为函数最后一条命令的返回值

echo "a + b = $res"
```
