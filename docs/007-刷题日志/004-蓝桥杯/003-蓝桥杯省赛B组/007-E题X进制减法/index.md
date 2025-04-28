# [蓝桥杯 2022 省 B] X 进制减法
链接: [[蓝桥杯 2022 省 B] X 进制减法](https://www.luogu.com.cn/problem/P8782)
## 题目描述

进制规定了数字在数位上逢几进一。

$X$ 进制是一种很神奇的进制，因为其每一数位的进制并不固定！例如说某种 $X$ 进制数，最低数位为二进制，第二数位为十进制，第三数位为八进制，则 $X$ 进制数 `321` 转换为十进制数为 `65`。

现在有两个 $X$ 进制表示的整数 $A$ 和 $B$，但是其具体每一数位的进制还不确定，只知道 $A$ 和 $B$ 是同一进制规则，且每一数位最高为 $N$ 进制，最低为二进制。请你算出 $A-B$ 的结果最小可能是多少。

请注意，你需要保证 $A$ 和 $B$ 在 $X$ 进制下都是合法的, 即每一数位上的数字要小于其进制。

## 输入格式

第一行一个正整数 $N$，含义如题面所述。

第二行一个正整数 $M_{a}$，表示 $X$ 进制数 $A$ 的位数。

第三行 $M_{a}$ 个用空格分开的整数，表示 $X$ 进制数 $A$ 按从高位到低位顺序各个数位上的数字在十进制下的表示。

第四行一个正整数 $M_{b}$，表示 $X$ 进制数 $B$ 的位数。

第五行 $M_{b}$ 个用空格分开的整数，表示 $X$ 进制数 $B$ 按从高位到低位顺序各个数位上的数字在十进制下的表示。

请注意，输入中的所有数字都是十进制的。

## 输出格式

输出一行一个整数，表示 $X$ 进制数 $A-B$ 的结果的最小可能值转换为十进制后再模  $1000000007$ （即 $10^9+7$ ）的结果。

## 样例 #1

### 样例输入 #1

```
11
3
10 4 0
3
1 2 0
```

### 样例输出 #1

```
94
```

## 提示

**【样例说明】**

当进制为：最低位 $2$ 进制, 第二数位 $5$ 进制, 第三数位 $11$ 进制时, 减法得到的差最小。此时 $A$ 在十进制下是 $108$，$B$ 在十进制下是 $14$，差值是 $94$。

**【评测用例规模与约定】**

对于 $30 \%$ 的数据， $N \leq 10,M_{a}, M_{b} \leq 8$.

对于 $100 \%$ 的数据， $2 \leq N \leq 1000,1 \leq M_{a}, M_{b} \leq 10^5,A \geq B$。

蓝桥杯 2022 省赛 B 组 E 题。

# 题解
首先我们发现一个数的 X进制 即

$$
val = a_1,a_2,..a_n = \sum_{i=0}^{n}(a_{i} \times \prod_{j=0}^{i} n_{i})
$$
例如:

$$ (120)_{X}=1 \times(5 \times 2)+2 \times 2+0 \times 1=10+4+0=14 $$

显然直接贪心就可以了!

即 对于 arr[i] 与 brr[i] 最小的进制数 显然是 `max(max(arr[i], arr[j]) + 1, 2)`

这些tm的我都知道!

就是他们的对齐有tm的问题!

## #1

```C++
#include <cstdio>
#include <vector>

using namespace std;
const int mod = 1e9 + 7;

int main() {
    int n;
    scanf("%d", &n); // 最高进制
    int m;
    scanf("%d", &m);
    vector<long long> arr(m);
    for (int i = 0; i < m; ++i)
        scanf("%d", &arr[i]);
    
    int mlen = m;
    scanf("%d", &m);
    mlen = max(mlen, m);
    vector<long long> jz(mlen);
    vector<long long> brr(m);
    for (int i = 0; i < m; ++i)
        scanf("%d", &brr[i]);
    
    // 对齐错误, 八嘎八嘎八嘎 
    // 贪心, 一个位使用最小进制
    long long res = 0;
    
    for (int i = jz.size() - 1; i >= 0; --i) {
        if (i < brr.size() && i < arr.size()) {
            jz[i] = max(brr[i], arr[i]) + 1;
            if (jz[i] <= 2)
                jz[i] = 2;
        }
        else {
            if (i < arr.size())
                jz[i] = (arr[i] <= 1 ? 2 : arr[i] + 1);
            else
                jz[i] = (brr[i] <= 1 ? 2 : brr[i] + 1);
        }
    }
    
    long long a = arr[arr.size() - 1];
    long long b = brr[brr.size() - 1];
    long long qianzuo = 1;
    for (int i = arr.size() - 2, j = jz.size() - 1; i >= 0; --i, --j) {
        qianzuo *= jz[j];
        qianzuo %= mod;
        a = (a + arr[i] * qianzuo) % mod;
    }
    
    qianzuo = 1;
    for (int i = brr.size() - 2, j = jz.size() - 1; i >= 0; --i, --j) {
        qianzuo *= jz[j];
        qianzuo %= mod;
        b = (b + brr[i] * qianzuo) % mod;
    }
    
    printf("%lld\n", (a - b + mod) % mod);

    return 0;
} 
```


## #2
```C++
#include <cstdio>
#include <vector>

using namespace std;
const int mod = 1e9 + 7;
const int maxlen = 1e5 + 5;

int main() {
    int n;
    scanf("%d", &n); // 最高进制
    int ma, mb;
    vector<long long> arr(maxlen);
    vector<long long> brr(maxlen);
    vector<long long> jz(maxlen, 1);
    scanf("%d", &ma);
    for (int i = maxlen - 1, j = 0; j < ma; --i, ++j)
        scanf("%lld", &arr[i]);
    
    scanf("%d", &mb);
    for (int i = maxlen - 1, j = 0; j < mb; --i, ++j)
        scanf("%lld", &brr[i]);
    
    // 还是tm的对齐问题艹 
    int nowmaxlen = max(ma, mb);
    for (int i = maxlen - 1, j = 0; j < nowmaxlen; --i, ++j) {
        jz[i] = max(max(arr[i], brr[i]) + 1, 2LL);
//        printf("[%lld %lld %lld] ", arr[i], brr[i], jz[i]);
    }
//    printf("\n");
        
    
    long long q = 1;
    long long a = arr[maxlen - ma];
    long long b = brr[maxlen - mb];
    for (int i = maxlen - ma + 1, j = maxlen - nowmaxlen; i < maxlen; ++i, ++j) {
        q = (q * jz[j]) % mod;
        a = (a + arr[i] * q) % mod;
    }
    
    q = 1;
    for (int i = maxlen - mb + 1, j = maxlen - nowmaxlen; i < maxlen; ++i, ++j) {
        q = (q * jz[j]) % mod;
        b = (b + brr[i] * q) % mod;
    }
    
    printf("%lld\n", (a - b + mod) % mod);
/*
11
3
3 2 1
2
2 1

1000
10
128 666 777 888 999 111 222 333 444 555
2
888 888
336005325 x
     */
    return 0;
} 
```

## #3 AC

果然是 对齐问题!

请多画图再设计如何存储数据!!!
```C++
对于

高位 -> 低位
1 2 3 4
2 3

我们需要低位对齐
即:
4 3 2 1
3 2

然后才可以确定位的进制数!
```



```C++
#include <cstdio>
#include <vector>

using namespace std;
const int mod = 1e9 + 7;
const int maxlen = 1e5 + 5;

int main() {
    int n;
    scanf("%d", &n); // 最高进制
    int ma, mb;
    vector<long long> arr(maxlen);
    vector<long long> brr(maxlen);
    vector<long long> jz(maxlen, 1);
    scanf("%d", &ma);
    for (int i = maxlen - ma; i < maxlen; ++i)
        scanf("%lld", &arr[i]);
    
    scanf("%d", &mb);
    for (int i = maxlen - mb; i < maxlen ; ++i)
        scanf("%lld", &brr[i]);
    
    // 还是tm的对齐问题艹 
    int nowmaxlen = max(ma, mb);
    for (int i = maxlen - 1, j = 0; j < nowmaxlen; --i, ++j) {
        jz[i] = max(max(arr[i], brr[i]) + 1, 2LL);
    }
        
    
    long long q = 1;
    long long a = arr[maxlen - 1];
    long long b = brr[maxlen - 1];
    for (int i = maxlen - 2, j = 1; j < ma; --i, ++j) {
        q = (q * jz[i + 1]) % mod;
        a = (a + arr[i] * q) % mod;
    }
    
    q = 1;
    for (int i = maxlen - 2, j = 1; j < mb; --i, ++j) {
        q = (q * jz[i + 1]) % mod;
        b = (b + brr[i] * q) % mod;
    }
    
    printf("%lld\n", (a - b + mod) % mod);

    return 0;
} 
```

## 标准答案

```C++
#include<bits/stdc++.h>
#define int long long
using namespace std;
const int N=1e5+5,MOD=1e9+7;
int n,ma,mb,x,y;
int a[N],b[N],tmp[N],final[N];
//a和b是输入数组，tmp存每一位的最小权值，final存比这位低的位上的进制之积
signed main(){
    cin>>n>>ma;
    for(int i=1;i<=ma;i++){
        int x;
        cin>>x;
        a[ma-i+1]=x;
    }
    cin>>mb;
    for(int i=1;i<=mb;i++){
        int x;
        cin>>x;
        b[mb-i+1]=x;
    }
    for(int i=max(ma,mb);i>=1;i--)
        tmp[i]=max(max(a[i],b[i])+1,1LL*2);//算出每一位的权值
    final[1]=1;
    for(int i=2;i<=max(ma,mb);i++)
        final[i]=(final[i-1]*tmp[i-1])%MOD;//前缀和维护乘积
    for(int i=max(ma,mb);i>=1;i--){
        x=x+final[i]*a[i];
        y=y+final[i]*b[i];
        //加
        x%=MOD,y%=MOD;
        //每次取模
    }
    int ans=x-y;
    //答案可能为负数
    while(ans<0)
        ans+=MOD;
    ans%=MOD;
    cout<<ans;
    return 0;
}
```
