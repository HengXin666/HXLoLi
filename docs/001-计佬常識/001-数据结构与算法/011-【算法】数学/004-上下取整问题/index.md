# 上下取整问题
## 代码实现
### 上取整

```C++
double a = 6.4;
double b = 1.3;

int fxxk(double a, double b) {
    return (a / b - (int)(a / b) > 0.0) ? (int)(a / b) + 1 : (int)(a / b);
}

// 求 a / b 上取整
int res = fxxk(a, b);
```

### 下取整

```C++
int res = a / b; // 直接除即可
```

## 上取整转化为下取整
因为在一般的程序语言中, 整数相除, 结果是一个 **下取整** 后的整数.

如果使用上取整函数`ceil()`, 但是在C++中, 它的返回值是一个`浮点数`, 涉及到浮点计算, 对性能有损耗.

所以有以下公式 $$\left\lceil\frac{k}{m}\right\rceil=\left\lfloor\frac{k-1}{m}\right\rfloor+1$$

此处我们需要讨论一下 $\frac{k}{m}$ 是否可以被整除

1. 如果 $\frac{k}{m}$ **可以** 被整除
    
    - 那么有 $\frac{k-1}{m}$ 一定 **不能** 被整除

      - 则 $\frac{k}{m}=\left\lceil\frac{k}{m}\right\rceil=\left\lfloor\frac{k-1}{m}\right\rfloor + 1$

2. 如果 $\frac{k}{m}$ **不可以** 被整除
    
    - 那么有 $\left\lfloor\frac{k-1}{m}\right\rfloor < \frac{k-1}{m} < \frac{k}{m} \ 且有 \ \frac{k}{m} - \left\lfloor\frac{k-1}{m}\right\rfloor < 1$

      - 则 $\left\lceil\frac{k}{m}\right\rceil = \left\lfloor\frac{k-1}{m}\right\rfloor + 1$

证毕!

附加有相关应用的题目: (*实际使用的时候直接采用代数法即可, 具体可以见下面, 有写*)
- [100228. 执行操作使数据元素之和大于等于 K](../../../../007-刷题日志/002-力扣/003-未分类题解/035-执行操作使数据元素之和大于等于K/index.md)