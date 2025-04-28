# 质数筛

## 埃拉托斯特尼筛法
### 1. 求最小质因数

> LPF[i] 是 i 的最小质因数

```C++
constexpr int N = 1e6 + 1;
int LPF[N];

int __init__ = []() -> int {
    LPF[1] = 1;
    for (int i = 2; i < N; ++i)
        if (!LPF[i])
            for (int j = i; j < N; j += i)
                if (!LPF[j])
                    LPF[j] = i;
    return 0;
}();
```

### 2. 求范围内所有质数
```C++
// 平方根优化

vector<int> prime;
bool is_prime[N];

void Eratosthenes(int n) {
    is_prime[0] = is_prime[1] = false;
    for (int i = 2; i <= n; ++i) 
        is_prime[i] = true;
    // i * i <= n 说明 i <= sqrt(n)
    for (int i = 2; i * i <= n; ++i) {
        if (is_prime[i])
        for (int j = i * i; j <= n; j += i) 
            is_prime[j] = false;
    }
    for (int i = 2; i <= n; ++i)
        if (is_prime[i])
            prime.push_back(i);
}
```

时间复杂度: $O(N\log\log{N})$

## 2. 线性筛

```C++
vector<int> pri;
bool not_prime[N];

void pre(int n) {
    for (int i = 2; i <= n; ++i) {
        if (!not_prime[i]) {
            pri.push_back(i);
        }
        for (int pri_j : pri) {
            if (i * pri_j > n) 
                break;
            not_prime[i * pri_j] = true;
            if (i % pri_j == 0) {
                // i % pri_j == 0
                // 换言之，i 之前被 pri_j 筛过了
                // 由于 pri 里面质数是从小到大的，所以 i 乘上其他的质数的结果一定会被
                // pri_j 的倍数筛掉，就不需要在这里先筛一次，所以这里直接 break
                break;
            }
        }
    }
}
```

时间复杂度: $O(N)$