# Promise 对象
## Promise 简介
Promise 对象代表了未来将要发生的事件，用来传递异步操作的消息，其状态不受外界影响。Promise对象代表一个异步操作，有三种状态：

- pending: 初始状态，不是成功或失败状态。
- fulfilled: 意味着操作成功完成。
- rejected: 意味着操作失败。

只有异步操作的结果，可以决定当前是哪一种状态，任何其他操作都无法改变这个状态。这也是Promise 这个名字的由来，它的英语意思就是`承诺`，表示其他手段无法改变。

一旦 Promise 对象从初始状态改变，就不会再变，任何时候都可以得到这个结果。Promise 对象的状态改变，只有两种可能：从 Pending 变为 Resolved 和从 Pending 变为 Rejected。只要这两种情况发生，状态就凝固了，不会再变了，会一直保持这个结果。

## Promise 应用

```js
let promise = new Promise(function (resolve, reject) {
    // 异步处理
    // 处理结束后、调用resolve 或 reject
});
promise.then(function (result) {
    // result的值是上面调用resolve(...)方法传入的值.可以对该结果进行相应的处理
});
promise.catch(function (error) {
    // error的值是上面调用reject(...)方法传入的值.可以对该结果进行相应的处理
});

// 链式调用
let promise = new Promise(function (resolve, reject) {
    // 异步处理
    // 处理结束后、调用resolve 或 reject
}).then(function (result) {
    // result的值是上面调用resolve(...)方法传入的值.可以对该结果进行相应的处理
}).catch(function (error) {
    // error的值是上面调用reject(...)方法传入的值.可以对该结果进行相应的处理
});
```

Promise 构造函数包含一个参数和一个带有 resolve（解析）和 reject（拒绝）两个参数的回调。在回调中执行一些操作（例如异步），如果一切都正常，则调用 resolve，否则调用 reject。

示例
```js
function calculate(a, b) {
    let promiser = new Promise(function (resolve, reject) {
        if (b === 0) {
            reject(new Error("不能除以0"));
        } else {
            setTimeout(function () {
                resolve(a / b);
            } , 2000);
        }
    });

    promiser.then(function (resolve) {
        console.log(resolve);
    });

    promiser.catch(function (reject) {
        console.log(reject);
    });
}

calculate(1, 0);
```