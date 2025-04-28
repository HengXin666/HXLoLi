# 速成!!!
## 异cpp
在 java 中, 只能在逻辑判断语句中使用布尔值作为最终的结果:

```java
if (!true)   // 合法

if (!114514) // 不行, !只能对bool操作

if (114514)  // 不行, if 只认 bool
```

除此之外基本上和Cpp无异了

## if

依旧有短路求值!

> ### 逻辑短路
>
> 逻辑与短路
>
> > 使用逻辑与衔接的多个条件中，只要其中一个条件为假，那么该条件之后的所有条件将得不到执行，从而形成逻辑与短路。
>
> 逻辑或短路
> 
> > 使用逻辑或衔接的多个条件中，只要其中一个条件为真，那么该条件之后的所有条件将得不到执行，从而形成逻辑或短路。


```java
if (true || (100 % 2 == 0 && 101 / 100 == 0) || false) // 为真
    ;
else if (a == 114514)
    ;
else
    ;
```

## 三元一次运算符（条件 ? 表达式1 : 表达式2）

```java
int a = 1, b 2= 2;
int max = b > a ? b : a;
```


## switch

```java
switch (a) {
    case 1:
        
        break;
    case 2:
        
        break;
    default:
        break;
}
```

## for

```java
// 2 - 100 内的所有质数
for (int i = 1; i <= 100; ++i) {
    for (int j = 2; j < i; ++j) {
        if (i % j == 0) {
            System.out.println(i + "是质数");
            break;
        }
    }
}

// foreach循环 同 cpp 吧
for (type element : array) {
    // code block
}
```

## while

```java
while (true) {
    ;
}
```

## do-while

```java
do {
    continue;  
} while (false);
```
