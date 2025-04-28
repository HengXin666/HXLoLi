# 外星密码
链接: [P1928 外星密码](https://www.luogu.com.cn/problem/P1928)
## 题目描述

有了防护伞，并不能完全避免 2012 的灾难。地球防卫小队决定去求助外星种族的帮助。经过很长时间的努力，小队终于收到了外星生命的回信。但是外星人发过来的却是一串密码。只有解开密码，才能知道外星人给的准确回复。解开密码的第一道工序就是解压缩密码，外星人对于连续的若干个相同的子串 $\texttt{X}$ 会压缩为 $\texttt{[DX]}$ 的形式（$D$ 是一个整数且 $1\leq D\leq99$ ），比如说字符串 $\texttt{CBCBCBCB}$ 就压缩为 $\texttt{[4CB]}$ 或者 $\texttt{[2[2CB]]}$，类似于后面这种压缩之后再压缩的称为二重压缩。如果是 $\texttt{[2[2[2CB]]]}$ 则是三重的。现在我们给你外星人发送的密码，请你对其进行解压缩。

## 输入格式

输入一行，一个字符串，表示外星人发送的密码。

## 输出格式

输出一行，一个字符串，表示解压缩后的结果。

## 样例 #1

### 样例输入 #1

```
AC[3FUN]
```

### 样例输出 #1

```
ACFUNFUNFUN
```

## 提示

【数据范围】

对于 $50\%$ 的数据：解压后的字符串长度在 $1000$ 以内，最多只有三重压缩。

对于 $100\%$ 的数据：解压后的字符串长度在 $20000$ 以内，最多只有十重压缩。保证只包含数字、大写字母、`[` 和 `]`。

# 代码
## 我的

模拟解析过程, 十分复杂, 但是这个是力扣需要

```C++
#include <iostream>
#include <string>
#include <stack>
#include <functional>
using namespace std;

bool tag = 0;

int find_right(int bg, const string& mm) {
    stack<char> S;
    for (int i = bg; i < mm.size(); ++i) {
        if (mm[i] == '[') {
            S.push(mm[i]);
            tag = 1;
        } else if (mm[i] == ']') {
            if (!S.size())
                return i;
            S.pop();
        }
    }
    return -1; // sb 
}

int main() {
    
    string mm;
    cin >> mm;
    string res;
    
    function<string(int, int)> dfs = [&](int bg, int en) { // 获取[]内容 
        string res;
        if (bg >= en)
            return res;
        
        if (mm[bg] == '[') {
            // AC[3FUN]
            // ABF[2RA[3A]B[2CD]]
            // A[2B[2C][2D]]
            
            // A[2[2[2B]]C[2[2D]]]
            // ABBBBCDDDDBBBBCDDDD

/*
A
    [2
        [2
            [
                2B
            ]
        ]
        C
        [2
            [
                2D
            ]
        ]
    ]
*/

            // A[11[11B[11C[6[D]]]]]

            // [10DA[7IS]]

            // [21abc]
            // 0123456
            // len = 6 - 0 - 3

            // GDWKUW[49C[50O]OY][37W[82WOTYG]]M
            // [50O]
            
            // [21A[11B]C[13D]]

            // 解析后面的数字
            int g = -1;
            if (bg + 2 < mm.size()) {
                if (mm[bg + 2] >= '0' && mm[bg + 2] <= '9') {
                    g = mm[bg + 2] - '0';
                }
            }
            int sw = mm[bg + 1] - '0';
            string tmp;
            int r = find_right(bg + 1, mm);
            
            if (tag) { // 有子串
                tag = 0; 
                tmp = dfs(bg + 1, r);
            } else {
                tmp = mm.substr(bg + 2 + (g >= 0), r - bg - 2 - (g >= 0));
            }
            
            // sw g
            for (int i = ((g >= 0) ? g + sw * 10: sw); i > 0; --i) {
                res += tmp;
            }
            
            return res + dfs(r + 1, en);
        }
        
        for (int i = bg; i < mm.size(); ++i) {
            if (mm[i] >= 'A' && mm[i] <= 'Z') {
                res += mm[i];
            } else if (mm[i] == '[')
                return res + dfs(i, en);
        }
        return res;
    };
    
    cout << dfs(0, mm.size()) << "\n";
    
    return 0;
}
```

## AC

边读边解析

```C++
#include <iostream>
#include <string>
#include <functional>
using namespace std;

int main() {
    function<string()> dfs = [&]() {
        int n;
        string str = "", tmp;
        char x;
        while (cin >> x) {
            if (x == '[') {
                cin >> n;
                tmp = dfs();
                while (n--)
                    str += tmp;
            } else if (x == ']') {
                return str;
            } else {
                str += x;
            }
        }
        return str;
    };
    
    cout << dfs() << "\n";
    
    return 0;
}
```
