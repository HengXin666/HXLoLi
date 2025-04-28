# rt
## 题目描述
程序猿圈子里正在流行一种很新的简写方法：

对于一个字符串，只保留首尾字符，将首尾字符之间的所有字符用这部分的长度代替。

例如`internationalization`简写成`i18n`，`Kubernetes`简写成`K8s`，`Lanqiao`简写成`L5o`等。

在本题中，我们规定长度大于等于K的字符串都可以采用这种简写方法（长度小于 $k$ 的字符串不配使用这种简写）。

给定一个字符串 $S$ 和两个字符 $C_1$ 和 $C_2$，请你计算 $S$ 有多少个以 $C_1$ 开头 $C_2$ 结尾的子串可以采用这种简写?

### 输入格式
- 第一行包含一个整数 $k$。
- 第二行包含一个字符串 $S$ 和两个字符 $C_1$ 和 $C_2$。

### 输出格式
- 一个整数代表答案。

输入:

```C++
4
abababdb a b
```

输出

```C++
6
```

说明: 符合条件的子串如下所示，中括号内是该子串

```C++
[abab]abdb
[ababab]db
[abababdb]
ab[abab]db
ab[ababdb]
abab[abdb]
```

数据范围: $ 2 \le k \le |S| \le 5 \times 10^5$

# 题解
## 暴力

```C++
#include <iostream>
#include <string>
#include <vector>
#include <set>

using namespace std;

int main() {
    ios::sync_with_stdio(0);
    cin.tie(0);
    cout.tie(0);
    int k;
    string str;
    cin >> k;
    cin >> str;
    char s, e;
    cin >> s >> e;
    int res = 0;
    vector<int> stt, enn;
    for (int i = 0; i < str.size(); ++i) {
        if (str[i] == s) {
            stt.push_back(i);
        } else if (str[i] == e) {
            enn.push_back(i);
        }
    }
    
    int edd = 0;
    for (int& i : stt) {
        bool tag = 1;
        for (int j = edd; j < enn.size(); ++j) {
            if (enn[j] - i + 1 >= k) {
                ++res;
                if (tag) {
                    edd = j;
                    tag = 0;
                }
            }
        }
    }
    
    cout << res << "\n";
    
    return 0;
}
```


## 双指针

请好好体会!, 类似于滑动窗口但是它不会缩小

```C++
#include <iostream>
#include <string>
#include <vector>
#include <set>

using namespace std;

int main() {
    ios::sync_with_stdio(0);
    cin.tie(0);
    cout.tie(0);
    int k;
    string str;
    cin >> k;
    cin >> str;
    char s, e;
    cin >> s >> e;
    long long res = 0;
    
    int L = 0, R = k - 1;
    long long c1 = 0;
    
    while (1) {
        if (R == str.size())
            break;
        
        if (str[L] == s) // 以 s 开头 
            ++c1;
        
        if (str[R] == e) // 以 e 结尾, 那么说明之前的 c1 个 以 s 开头的字符串可以配对
            res += c1; // 那么有效的就是 c1 个开头 字符串 
        
        ++L, ++R;
    }
    
    cout << res << "\n";
    
    return 0;
}
```
