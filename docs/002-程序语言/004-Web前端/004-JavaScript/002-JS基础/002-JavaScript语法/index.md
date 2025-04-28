# JavaScript 语法
## 数据类型

| 数据类型 | 说明 |
| --- | --- |
| undefined | `var msg;` 变量msg没有赋初始值，默认为undefined |
| null | 空值，与undefined值相同，但类型不同 |
| number | `var num = 10;` |
| boolean | `var valid = true;` |
| string | `var name ="张三"; var sex='男';` |
| object | `var obj = new Object(); var stu = {name: '张三', sex:'男'};` |

## 变量
### var 关键字定义变量
JavaScript 是一种弱类型语言（没有类型之分），因此，在定义的变量的时候统一使用var关键字来定义。在JavaScript中，变量也是严格区分大小写的

```js
var msg = 20;       // 赋值数字
msg = "字符串";      // 赋值字符串
msg = true;         // 赋值布尔值
msg = new Object(); // 赋值对象
```

### let 关键字定义变量

```js
let name = "张三";
let number = 10;
```

### var 与 let 的区别

注意, 如果val在函数内部声明，那么它是局部变量，只能在该函数内部访问

如果你在一个`<script>`标签内部使用`var`声明了一个变量，那么这个变量可以在同一HTML文件的其他`<script>`标签中访问，因为它是全局的。

```js
// {} 也是作用域
{
    let innerLet = "代码块内定义的let变量"; // let 作用域在{}内 的, 局部的
    var innerVar = "代码块内定义的var变量"; // var 定义的作用域在全局
}
console.log(innerVar);
console.log(innerLet);
```

## 字符串
### 定义字符串
在JavaScript 中，凡事使用单引号或者双引号引起来的内容都属于字符串。

```js
let name = "张三";  // 双引号表示的字符串
let sex = '男';    // 单引号表示的字符串
```

### 常用函数

基本上同`java`的字符串方法

```js
let str = "这是一个字符串";
console.log(str.length);                // 打印字符串的长度

let c = str.charAt(1);                  // 获取下标为1的字符，在JS中没有字符，因此结果是一个字符串
console.log(c);

let index = str.indexOf("个");          // 获取字符串中第一次出现"个"的下标
console.log(index);

let sub = str.substring(3, 6);;         // 获取字符串中位于区间[3, 6)之间的字符串
console.log(sub);

let arr = str.split("");                // 将字符串按照空白字符串进行分割，分割结果为字符串数组
console.log(arr);

let replaceStr = str.replace("一个", "");// 将字符串中的"一个"使用空白字符串替换
console.log(replaceStr);
```

## 数组
### 创建数组
```js
let 数组名 = new Array(数组长度);
let 数组名 = new Array(数组元素1, 数组元素2, ..., 数组元素n);
let 数组名 = [数组元素1, 数组元素2, ..., 数组元素n]; // 用 [ ] 的就是数组, 并且类型不用一致
```

### 数组元素赋值

```js
// 元素赋值
let numbers = new Array(10);// 创建了一个长度位10的数组
numbers[0] = 1;             // 通过下标为数组元素赋值
numbers[1] = 2;
numbers[0] = 3;             // 修改数组中的元素
```

### 数组常用方法
同java的方法
```js
// 常用方法
let num1 = [1, 2, 3]
let length = num1.push(4, 5);   // 一次放入多个元素至数组中
console.log("数组长度：" + length);

let num2 = [6, 7, 8];
let num3 = num1.concat(num2);   // 将数组num2与num1进行在新数组中进行拼接，num2在num1之后
console.log("拼接后：" + num3);

num3.splice(2, 1);              // 将数组num3从下标为2的位置删除1个元素
console.log("删除元素后：" + num3);

num3.splice(3, 2, 10, 20, 30);  // 将数组num3从下标为3的位置删除2个元素，然后将10,20,30从删除位置添加到数组中
console.log("删除元素的同时增加元素：" + num3)

let str = num3.join(",");       // 将数组num3中所有元素使用","拼接起来
console.log(str);
```

## 对象
### 声明
```js
let 对象名 = new Object(); // 创建对象

对象名.属性名1 = 属性值1; // 为对象添加属性, 不用事先声明字段
对象名.属性名2 = 属性值2;
...
对象名.属性名n = 属性值n;

let 对象名 = {   // 使用大括号创建对象
    属性名1: 属性值1, // 属性名和属性值的关系使用冒号表示，多个属性之间使用逗号分割开
    属性名2: 属性值2,
    ...
    属性名n: 属性值n;
};
```

示例

```js
let stu = new Object();
stu.name = "张三";
stu.sex = "男";
stu.age = 20;
console.log(stu);

let teacher = {
    name : '李刚',
    level: '教授',
    salary: 18000
};

console.log(teacher);
```
