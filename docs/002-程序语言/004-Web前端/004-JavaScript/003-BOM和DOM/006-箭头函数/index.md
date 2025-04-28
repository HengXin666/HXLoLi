# 箭头函数
箭头函数相当于Java中的 lambda 表达式，传递的依然是实现过程。

```js
function (参数) {
    // ...
}

变为
(参数) => {} // 如果只有一个参数可以省略 括号
```

示例 (例子直接改[Promise对象](../005-Promise对象/index.md)的示例了)
```js
function calculate(a, b) {
    let promiser = new Promise((resolve, reject) => {
        if (b === 0) {
            reject(new Error("不能除以0"));
        } else {
            setTimeout(() => {
                resolve(a / b);
            } , 2000);
        }
    }).then(resolve => {
        console.log(resolve);
    }).catch(reject => {
        console.log(reject);
    });
}

calculate(1, 0);
```

示例
```js
((a, b) => {
    setTimeout(() => {
        console.log(a / b);
    } , 1000);
})(11, 2); // 匿名函数
```