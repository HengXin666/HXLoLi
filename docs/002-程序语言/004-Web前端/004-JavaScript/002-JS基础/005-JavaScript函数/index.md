# JS 函数
## 函数的概念
函数是用于完成特定功能的语句块，类似于 Java 语言中的方法。函数分为系统函数和自定义函数。

## 系统函数
### 窗体函数

| 函数名 | 说明 |
| --- | --- |
| `alert("提示信息")` | 提示对话框 |
| `confirm("提示信息")` | 确认对话框 |
| `prompt("提示信息")` | 输入对话框 |

注: 输入对话框有一个返回值,该值即为输入的信息;如果用户没有进行输入而进行确认,那么结果为`空字符串`;如果用户进行取消操作,那么结果为`null`

### 数字相关函数

| 函数名 | 说明 |
| --- | --- |
| `parseInt()` | 将给定的字符串转换为整数 |
| `parseFloat()` | 将给定的字符串转换为浮点数 |
| `isNaN()` | 判断给定的值是否不是数字 |

示例
```js
// 在JavaScript中,parseInt函数能够将以数字开头的任意字符串转换为整数
let a = parseInt("12a3");
console.log(a)
// 在JavaScript中,parseFloat函数能够将以数字以及'.'号开头的任意字符串转换为浮点数
let b = parseFloat(".123a")
console.log(b)
let result = isNaN("123");
console.log(result)
```

### Math 类函数

| 方法 | 说明 | 示例 |
| --- | --- | --- |
| `ceil(数值)` | 向上取整 | `Math.ceil(2.1);` 结果为3 |
| `floor(数值)` | 向下取整 | `Math.floor(2.9);` 结果为2 |
| `round(数值)` | 取距离该数最近的数 | `Math.round(2.5);` 结果为3 |
| `random()` | 取[0,1)之间的随机数 | `Math.random();` 结果为0~1之间的浮点数 |

## 自定义函数
语法
```js
function 函数名(参数列表) { // 可以不用谢let, 直接写变量名就ok
    // 有需要就写return
}

// 调用
函数名(参数值1, 参数值2, ... , 参数值n);
```

示例
```js
function sum(a, b)
{
    return a + b;
}

function show()
{
    console.log("这是JavaScript中的方法")
}

{
    show();
    let result = sum(1, 2);
    console.log(result);
}
```

高级示例
```js
/**
* 在JavaScript中，一个函数的返回值也可以是一个函数
* @param a
* @param b
* @param c
* @returns {function(*): number}
*/
function calculate(a, b, c)
{
    let result = a * b;
    return function (d) {
        return result + c * d;
    }
}

// 此时需要注意的是，calculate函数执行后得到的结果是一个函数，也就是说，在JavaScript中，
// 变量可以存储一个函数，这种情况，我们把这个变量当作函数使用即可
let s = calculate(1, 2, 3);
let num = s(4); // 再次调用函数，得到计算结果
console.log(num);

// 闭包，说白了，就是一个函数能够记住和访问其创建时所在的词法环境，即使在它被创建的词法环境已经消失后。
let n = calculate(1, 2, 3)(4); // 函数调用
console.log(n);
```

## 元素事件与函数

| 名称 | 说明 |
| --- | --- |
| `click` | 鼠标左键单击元素 |
| `blur` | 元素失去焦点 |
| `focus` | 元素获得焦点 |
| `keydown` | 键盘按键被按下 |
| `keyup` | 键盘按键被按下后释放 |
| `keypress` |键盘按键按下不论释放与否都生效|
| `mouseover` | 鼠标移动至元素上 |
| `mouseout` | 鼠标移动至元素外 |
| `change` | (失去焦点或回车)元素的内容发生改变 |
| `input` | (一更新就)元素的内容发生改变 |

开启元素事件只需要在事件名前面加上`on`即可，关闭元素事件只需要在事件名前面加上`off`即可。

示例: 注意名称 on + 事件名, 表示开始监听, 里面可以调用(自定义)函数
```html
<!DOCTYPE html>
<html lang="en">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>元素事件与函数</title>
</head>

<body>
    <!-- click事件 -->
    <button onclick="alert('点击了按钮!')">Click me</button>

    <!-- blur事件 -->
    <input type="text" onblur="console.log('元素失去焦点');" />

    <!-- focus事件 -->
    <input type="text" onfocus="console.log('元素获得焦点');" />

    <!-- mouseover事件 -->
    <div onmouseover="console.log('鼠标移动至元素上');">鼠标移动至元素上</div>

    <!-- mouseout事件 -->
    <div onmouseout="console.log('鼠标移动至元素外');">鼠标移动至元素外</div>

    <!-- change事件 -->
    <select onchange="console.log('元素的内容发生改变');">
        <option value="option1">Option 1</option>
        <option value="option2">Option 2</option>
    </select>
    <input type="text" onchange="console.log('元素的内容按下回车或失去焦点则发生改变');" />

    <!-- input事件 -->
    <input type="text" oninput="console.log('时刻元素的内容发生改变');" />

</body>

</html>
```

当然也可以这样: (有些知识还没学)
```js
// keydown事件
document.addEventListener("keydown", function(event) {
    console.log("Key pressed: " + event.key);
});

// keyup事件
document.addEventListener("keyup", function(event) {
    console.log("Key released: " + event.key);
});
```