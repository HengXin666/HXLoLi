# JS 流程控制语句
## if
同 c/c++/java

```js
let a = 10;
if (typeof a === "number") {
    console.log("变量a是一个数字")
}
else {
    console.log("变量a不是一个数字")
}
```

## switch
同 c/c++/java

```js
let a = 10;

switch (a % 3) {
    case 1:
        console.log("变量a与3求模的结果是1")
        break
    case 2:
        console.log("变量a与3求模的结果是2")
        break;
    default:
        console.log("变量a能够被3整除")
}
```

## for
同 c/c++/java, 注意不要声明为`int`啦~
```js
for(let i = 0; i < 10; i++)
{
    console.log(i);
}
```

### for-in
如下: *不同于py是自己从arr里面取值*
```js
let arr = [1, 2, 3, 4, 5];
for(let prop in arr)
{ 
    // 对于数组来说,使用for-in循环就是遍历数组的下标
    console.log(prop + "=>" + arr[prop])
}
console.log("=====================")

let stu =   {
    name: '李四',
    sex: '男',
    age : 20,
    score: 86
};

for(let prop in stu)
{
    // 对于对象来说,使用for-in循环就是遍历对象的属性
    // 对象的属性取值除了使用'.'操作符外,还可以使用中括号来取值
    console.log(prop + "=>" + stu[prop]);
}

console.log("=====================")
console.log(stu.name);
console.log(stu['name']);
```

## while
同 c/c++/java

```js
let num = 0;

while (num++ < 10)
{
    console.log(num);
}
```

## do-while
同 c/c++/java

```js
let num = 10;

do
{
    console.log(num--);
} while (num >= 0)
```