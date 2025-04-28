# JS 运算符
只需要注意3个东西

1. 整数 除以 整数 得到的可能是浮点数
2. `===` 判断值相等 && 类型相等
3. `!==` 判断值不相等 || 不类型相等

其他的和 C/C++, Java 同

示例:
```js
let a = 1, b = 2;
console.log(a++);
console.log(a);
console.log(++a);
console.log(a);
a += b;
console.log(a); // a = 5

//在Java中两个整数相除所得的结果一定是整数;但是在JavaScript中,
//两个整数相除,得到的结果可能是浮点数
let result = a / b;
console.log(result)
console.log( a % b);

// js的比较
let c = "2";
console.log( b == c);       //两个等号进行比较,只比较内容是否相同
console.log( b === c);      //三个等号进行比较,比较内容是否相同的同时还要检查数据类型是否一致

console.log( b != c);       //只有一个等号的不等于, 只比较内容是否相同
console.log( b !== c);      //有两个等号的不等于,比较内容是否相同的同时还要检查数据类型是否一致

let s1 = a > 1 && b === c;  //逻辑与    // f
let s2 = a > 1 || b === c;  //逻辑或    // t
let s3 = !a > 1             //逻辑非    // !t --> f

console.log(s1 + " " + s2 + " " + s3);
```
