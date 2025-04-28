# [蓝桥杯 2022 国 B] 最大数字
链接: [P8801 [蓝桥杯 2022 国 B] 最大数字](https://www.luogu.com.cn/problem/P8801)
## 题目描述

给定一个正整数 $N$。你可以对 $N$ 的任意一位数字执行任意次以下 2 种操作:

1. 将该位数字加 $1$。如果该位数字已经是 $9$，加 $1$ 之后变成 $0$。

2. 将该位数字减 $1$。如果该位数字已经是 $0$，减 $1$ 之后变成 $9$。

你现在总共可以执行 $1$ 号操作不超过 $A$ 次，$2$ 号操作不超过 $B$ 次。

请问你最大可以将 $N$ 变成多少?

## 输入格式

第一行包含 3 个整数：$N$，$A$，$B$ 。

## 输出格式

一个整数代表答案。

## 样例 #1

### 样例输入 #1

```
123 1 2
```

### 样例输出 #1

```
933
```

## 提示

**【样例说明】**

对百位数字执行 $2$ 次 $2$ 号操作，对十位数字执行 $1$ 次 $1$ 号操作。

**【评测用例规模与约定】**

对于 $30 \%$ 的数据， $1 \leq N \leq 100 ; 0 \leq A, B \leq 10$ 

对于 $100 \%$ 的数据, $1 \leq N \leq 10^{17} ; 0 \leq A, B \leq 100$ 

蓝桥杯 2022 国赛 B 组 D 题。

# 题解
## 我代码

直接贪心
```C++
#include <cstdio>
#include <iostream>
#include <vector>
#include <string> 

using namespace std;

int main() {
    string s;
    int a, b;
    cin >> s >> a >> b;
    // 先从最高位开始改
    for (int i = 0; i < s.size(); ++i) {
        if (s[i] - '0' <= 4 && b > s[i] - '0') {
            b -= s[i] - '0' + 1;
            s[i] = '9';
        } else if (a > 0) {
            if (s[i] - '0' < 9 && a > 9 - (s[i] - '0')) {
                a -= 9 - (s[i] - '0');
                s[i] = '9';
            } else {
                while (a && 9 - (s[i] - '0')) {
                    ++s[i];
                    --a;
                }
            }
        }
    }
    cout << s << '\n';
//    cout << "a: " << a << "  b: " << b << '\n';
    return 0;
}
```

## 2

发现贪心不行, 然后bfs搜索

```C++
#include <cstdio>
#include <iostream>
#include <vector>
#include <string> 

using namespace std;

string str;

// 无需回溯 
void bfs(int i, int a, int b, string& s) {
    str = max(str, s);
    if (i >= s.size() || !(a || b))
        return;
    
    if (b > s[i] - '0' && (s[i] - '0' < 9 && a > 9 - (s[i] - '0'))) {
        int t = s[i] - '0' + 1;
        int tt = 9 - (s[i] - '0');
        s[i] = '9';
        bfs(i + 1, a, b - t, s);
        bfs(i + 1, a - tt, b, s);
    }
    else if (b > s[i] - '0') { // b 的次数有 
        int t = s[i] - '0' + 1;
        s[i] = '9';
        bfs(i + 1, a, b - t, s);
    }
    else if (s[i] - '0' < 9 && a > 9 - (s[i] - '0')) {
        int t = 9 - (s[i] - '0');
        s[i] = '9';
        bfs(i + 1, a - t, b, s);
    }
    else if (s[i] - '0' < 9 && a > 0) {
        int t = a - (9 - (s[i] - '0'));
        if (t < 0)
            t = a;
            
        s[i] += t;
        bfs(i + 1, a - t, b, s);
    }
}

int main() {
    string s;
    int a, b;
    cin >> s >> a >> b;
    str = s;
    
    bfs(0, a, b, s);
    
    // 先从最高位开始改, 判断需要A的多, 还是B 
    
    cout << str << '\n';
//    cout << "a: " << a << "  b: " << b << '\n';
    return 0;
}
```

## AC

怎么写都不对, 感觉和题解没什么两样啊?为什么:

```C++
#include<bits/stdc++.h>
using namespace std;
const int N=1e5+10,INF=1e9;
int read()
{
    int x=0,f=1;char ch=getchar();
    while(ch<'0'||ch>'9')
    {
        if(ch=='-')
            f=-1;
        ch=getchar();
    }
    while(ch>='0'&&ch<='9')
        x=x*10+ch-'0',ch=getchar();
    return x*f;
}
string s,ans;//其实可以用字符串来存数的，甚至比long long更方便
int n,a,b; 
void dfs(int k,int c,int d,string str)
{
    if(k==n)
    {
        if(str>ans)//取最优解
            ans=str;
        return;
    }
    int x=9-str[k]+'0',y=str[k]-'0'+1;
    if(c+x<=a&&d+y<=b)//如果两种都能变为9
        str[k]='9',dfs(k+1,c+x,d,str),dfs(k+1,c,d+y,str);//两种都试一试
    else if(c+x<=a)//如果只有操作1能
        str[k]='9',dfs(k+1,c+x,d,str);
    else if(d+y<=b)//如果只有操作2能
        str[k]='9',dfs(k+1,c,d+y,str);
    else//如果都不能
        str[k]+=a-c,dfs(k+1,a,d,str);//把操作1用光
}
signed main()
{
    cin>>s,ans=s,n=s.size(),a=read(),b=read();
    dfs(0,0,0,s);
    cout<<ans;
}
```

## 二刷
贪心, 就是看看各种条件下下誰可以最大, 注意当`(n[i] == '4' && x >= 4 && y >= 4)`情况下, 两种都可以选择, 因此都要试一下(n <= 17) 所以直接dfs就可以了

```cpp
#include <iostream>
#include <string>
#include <functional>

using namespace std;

int main() {
    string n, tmp, res;
    int a, b;
    cin >> n >> a >> b;
    res = tmp = n;
    
    // 操作第 i 位数, 剩下 1 操作 a 次, 2 操作 b 次 
    function<void(int, int, int)> dfs = [&](int i, int x, int y) {
        if (i >= n.size()) {
            res = max(res, tmp);
            return;
        }
        
        if (n[i] == '4' && x >= 4 && y >= 4) {
            tmp[i] = '9';
            dfs(i + 1, x - 4, y);
            dfs(i + 1, x, y - 4);
        } else if ('9' - n[i] <= x && n[i] > '4' 
                    || n[i] < '4' && n[i] - '0' > y) { // 操作 1
            if ('9' - n[i] <= x) {
                x -= '9' - n[i];
                tmp[i] = '9';
            } else {
                tmp[i] += x;
                x= 0;
            }
            dfs(i + 1, x, y);
        } else if (tmp[i] - '0' <= y) { // 操作2 
            y -= tmp[i] - '0';
            tmp[i] = '9';
            dfs(i + 1, x, y);
        } else { // 做不了
            dfs(i + 1, x, y);
        }
    };
    
    dfs(0, a, b);
    
    cout << res;
    
    return 0;
}
```