# 模拟基础
## 输入输出
使用printf/scanf即可

但是如果需要读取整行, 则需要`getline(cin, str);`, 注意使用了`getline`就不能使用`cin`等, 防止出现读取到`\n`等八嘎

无终止输入, 使用`while (cin >> n){}`即可, 输入时候可以在控制台使用`ctrl+z`表示输入解释以测试代码.

注意: (特别是在`scnaf`中!)
1. %s 表示字符串。
2. %c 表示字符。
3. %lf 表示双精度浮点数 ( double )。
4. %lld 表示长整型 ( long long )。
5. %llu 表示无符号长整型 ( unsigned long long )，无符号整数不能读
入负数。
## 日期类
例如: [#73. 顺子日期(结果填空)](https://dashoj.com/d/lqbproblem/p/73)

小明特别喜欢顺子。顺子指的就是连续的三个数字：123、456 等。顺子日期指的就是在日期的 уyyymmdd 表示法中，存在任意连续的三位数是一个顺子的日期。例如 20220123 就是一个顺子日期，因为它出现了一个顺子：123; 而 20221023 则不是一个顺子日期，它一个顺子也没有。小明想知道在整个 2022 年份中，一共有多少个顺子日期。

### 模版

```C++
#include <cstdio>
#include <string>

using namespace std;

int main() {
    char str[5];
    int res = 0;
    for (int y = 2022; y <= 2022; ++y) {
        for (int m = 1; m <= 12; ++m) {
            for (int d = 1; d <= 31; ++d) {
                // 这里是模版
                if (!(m == 1 || m == 3 || m == 5 || m == 7 || m == 8 || m == 10 || m == 12)) {
                    if (m == 2) { // 闰年: 可以被400整除 或者 被4而不能被100整除的年
                        if (m % 400 == 0 || (y % 4 == 0 && y % 100 != 0)) { // 如果是润年 
                            if (d > 29)
                                break;
                        } else {
                            if (d > 28)
                                break;
                        }
                    } else {
                        if (d > 30)
                            break;
                    }
                }

                // 这里是需要模拟做的事情
                sprintf(str, "%02d%02d", m, d);
                int now = 1;
                
                for (int i = 1; i < 5; ++i) {
                    if (str[i - 1] + 1 == str[i]) {
                        ++now;
                        if (now >= 3) {
                            ++res;
                            printf("%s\n", str);
                            break;
                        }
                    } else {
                        now = 1;
                    }
                }
            }
        }
    }
    
    printf("%d", res);
    
    return 0;
} 
```

[P8748 [蓝桥杯 2021 省 B F题] 时间显示](https://www.luogu.com.cn/problem/P8748)
```C++
#include <cstdio>
#include <string>

using namespace std;

int main() {
    long long t;
    scanf("%lld", &t);
    t /= 1000; // 得到秒数
    long long now = 0;
    for (int y = 1970; y <= 50000000; ++y) {
        for (int m = 1; m <= 12; ++m) {
            for (int d = 1; d <= 31; ++d) {
                // 这里是模版
                if (!(m == 1 || m == 3 || m == 5 || m == 7 || m == 8 || m == 10 || m == 12)) {
                    if (m == 2) { // 闰年: 可以被400整除 或者 被4而不能被100整除的年
                        if (m % 400 == 0 || (y % 4 == 0 && y % 100 != 0)) { // 如果是润年 
                            if (d > 29)
                                break;
                        } else {
                            if (d > 28)
                                break;
                        }
                    } else {
                        if (d > 30)
                            break;
                    }
                }

                if (now + 24*60*60 >= t) {
                    // 为当天
                    int res = t - now; // 得到从 0 点开始的秒数
                    int hh = res / (60 * 60);
                    int mm = (res - hh * (60 * 60)) / 60;
                    int ss = res % 60;
                    printf("%02d:%02d:%02d\n", hh, mm, ss);
                    return 0;
                } else {
                    now += 24*60*60;
                }
            }
        }
    }
    return 0;
}
```

## 调试中常见错误
1. 注意 数据范围!!!定义单个变量能用 long long 就不要用 int ，能定义为double 就不要用 float 。
2. maxn 、 minn (最大值、最小值)或者 cnt (计数器)忘记 赋初始值。
3. 忘记删除用来检查代码的语句，比如添加了多余的打印操作。
4. 没理解题目的意思，就开始做题。要结合样例去理解题目。
5. 注意题目数据范围很大的时候， 考虑优化。
6. 数组开太大。比如: int arr[10000][10000] ，则 arr 的空间大小约为400M，远超题目的空间限制。
7. 边界条件考虑不充分。
8. 变量命名与c++自带的名称冲突(因此，慎用万能头文件)。如:int time; int max; int min等等。
9. 写了初始化函数 ，没有调用。
10. 使用函数时， 参数传错。
11. 用了一个新的知识点，但自己不太清楚，就用了。要注意，写的代码最好是你完全能明白的。
12. if里面的判断条件没想清楚，还继续往下写。
13. 循环里面条件写反，比如: for(int i = n;i > 0;i+ +)
14. 双重循环里面 i 和 i 写反，或者两个循环变量都写成 i。
15. 字符串 string 中，遍历 string 的操作最好用 for(int i = 0;i < s.size(); i++) ，不要用小于等于 i<= s.size() - 1