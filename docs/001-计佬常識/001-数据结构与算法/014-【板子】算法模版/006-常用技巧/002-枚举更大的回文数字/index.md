# 回文数字
## 1. 枚举更大的回文数字
- [866. 回文质数](https://leetcode.cn/problems/prime-palindrome/)


```C++
class Solution {
    using ll = long long;
public:
    int primePalindrome(int n) {
        auto isFk = [](int x) {
            for (int i = 2; i * i <= x; ++i)
                if (!(x % i))
                    return false;
            return x >= 2;
        };
        auto isSb = [](int x) -> bool {
            auto str = to_string(x);
            int l = 0, r = str.size() - 1;
            while (l < r)
                if (str[l++] ^ str[r--])
                    return false;
            return true;
        };
        auto nextSb = [&](int x) {
            auto str = to_string(x);
            while (1) {
                // 因为需要比 x 更大的回文数字
                // 显然右边对称到左边
                // 就可以构造 出可能的 >= x 的回文数字
                // 因此我们使用循环: 然后保证 nx >= x
                // 例如输入: 91999
                // -> 91919 (对称) -> if = false
                // -> 92019 (进位)
                // -> 92029 (对称) -> 92029 >= 91999
                for (int i = 0; i < str.size() / 2; ++i)
                    str[str.size() - 1 - i] = str[i];

                if (int nx = stoi(str); nx >= x)
                    return nx;

                int j = str.size() / 2 - !(str.size() & 1);
                while (str[j] == '9') {
                    str[j--] = '0';
                }
                ++str[j]; // 不可能越界, 因为 9999 的时候
                          // 已经从 nx >= x 退出了
            }
        };
        if (isSb(n) && isFk(n))
            return n;
        while (1) {
            if (isFk(n = nextSb(n)))
                return n;
            ++n;
        }
        return -1;
    }
};
```
