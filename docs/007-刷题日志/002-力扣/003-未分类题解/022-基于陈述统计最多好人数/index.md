# 2151. 基于陈述统计最多好人数
第 277 场周赛
Q4
1980 困难

游戏中存在两种角色：

- 好人：该角色只说真话。
- 坏人：该角色可能说真话，也可能说假话。
- 给你一个下标从 0 开始的二维整数数组 statements ，大小为 n x n ，表示 n 个玩家对彼此角色的陈述。具体来说，statements[i][j] 可以是下述值之一：

- 0 表示 i 的陈述认为 j 是 坏人 。
- 1 表示 i 的陈述认为 j 是 好人 。
- 2 表示 i 没有对 j 作出陈述。
另外，玩家不会对自己进行陈述。形式上，对所有 0 <= i < n ，都有 statements[i][i] = 2 。

根据这 n 个玩家的陈述，返回可以认为是 好人 的 最大 数目。

## 示例 1：
输入：statements = [[2,1,2],[1,2,2],[2,0,2]]<br>
输出：2<br>
解释：每个人都做一条陈述。<br>
- 0 认为 1 是好人。
- 1 认为 0 是好人。
- 2 认为 1 是坏人。
以 2 为突破点。
- 假设 2 是一个好人：
    - 基于 2 的陈述，1 是坏人。
    - 那么可以确认 1 是坏人，2 是好人。
    - 基于 1 的陈述，由于 1 是坏人，那么他在陈述时可能：
        - 说真话。在这种情况下会出现矛盾，所以假设无效。
        - 说假话。在这种情况下，0 也是坏人并且在陈述时说假话。
    - 在认为 2 是好人的情况下，这组玩家中只有一个好人。
- 假设 2 是一个坏人：
    - 基于 2 的陈述，由于 2 是坏人，那么他在陈述时可能：
        - 说真话。在这种情况下，0 和 1 都是坏人。
            - 在认为 2 是坏人但说真话的情况下，这组玩家中没有一个好人。
        - 说假话。在这种情况下，1 是好人。
            - 由于 1 是好人，0 也是好人。
            - 在认为 2 是坏人且说假话的情况下，这组玩家中有两个好人。
在最佳情况下，至多有两个好人，所以返回 2 。<br>
注意，能得到此结论的方法不止一种。<br>
## 示例 2：
输入：statements = [[2,0],[0,2]]<br>
输出：1<br>
解释：每个人都做一条陈述。<br>
- 0 认为 1 是坏人。
- 1 认为 0 是坏人。
以 0 为突破点。
- 假设 0 是一个好人：
    - 基于与 0 的陈述，1 是坏人并说假话。
    - 在认为 0 是好人的情况下，这组玩家中只有一个好人。
- 假设 0 是一个坏人：
    - 基于 0 的陈述，由于 0 是坏人，那么他在陈述时可能：
        - 说真话。在这种情况下，0 和 1 都是坏人。
            - 在认为 0 是坏人但说真话的情况下，这组玩家中没有一个好人。
        - 说假话。在这种情况下，1 是好人。
            - 在认为 0 是坏人且说假话的情况下，这组玩家中只有一个好人。
在最佳情况下，至多有一个好人，所以返回 1 。 <br>
注意，能得到此结论的方法不止一种。<br>

# 题解
## 我的代码

总是差点...

有空再写qwq..

```C++
class Solution {
public:
    // 不能贪心吗? 不能 因为都说坏人的情况下, 至少会有1个好人, 因为求的是好人的最大数量
    int maximumGood(vector<vector<int>>& statements) { // 0坏 1好 2略
        int res = 0;
        int tmp_max = 0;

        // 假设是i好人 / 假设是坏人
        map<int, int> arr;  // 人 - 当前属性
                            // 如果不符逻辑就return回溯
        function<void(int, int)> dfs =
        [&](int i, int tag) {
            if (i >= (int)statements.size()) {
                res = max(tmp_max, res);
                return;
            }

            // i 是好人
            arr.insert(make_pair(i + 1, 1)); // 0坏 1好
            map<int, int> tmp_map = arr;
            // 把i观点下的全部内容进行添加
            // 如果是好人, 要保证之前说的也是好人
            // 如果是坏人, 之前不能被认为是好人
            if (i >= 0)
            for (int j = 0; j < statements[i].size(); ++j) {
                if (statements[i][j] == 2)
                    continue;
                
                auto it = arr.find(j);
                // 不符合逻辑:
                // 1. 好人说 j 是 坏人, 但前面假设它是好人
                // 2. 好人说 j 是 好人, 但前面假设它是坏人
                if (it == arr.end()) {
                    // 观点不存在, 添加观点
                    tmp_map.insert(make_pair(j, statements[i][j]));
                }
                else
                    if (it->second != statements[i][j]) { // !0
                        // 它说it是坏人, 但是之前已经假设it是好人了
                        // 所以不成立
                        return;
                    }
            }
            tmp_map.swap(arr);
            ++tmp_max;
            dfs(i + 1, 1);
            --tmp_max;
            tmp_map.swap(arr);
            arr.erase(i + 1);

            // i 是坏人
            arr.insert(make_pair(i + 1, 0)); // 0坏 1好
            dfs(i + 1, 0);
            arr.erase(i + 1);
        };

        dfs(-1, -1);

        return res;
    }
};
```


```C++
class Solution {
    bool findVector(vector<bool>& runed) {
        int n = runed.size();
        for (int i = 0; i < n; ++i)
            if (!runed[i])
                return false;
        return true;
    }

    // void fun(int i, map<int, int>& arr, vector<vector<int>>& statements, vector<bool>& runed, function<void(int, int)>& dfs) {
    //         // 假设 i 是 好人
    //         map<int, int> tmp_map = arr;
    //         arr.insert(make_pair(i, 1)); // i 是好人
    //         runed[i] = 1;
    //         for (int j = 0; j < statements[i].size(); ++j) {
    //             if (statements[i][j] == 2 || arr.find(j) == arr.end())
    //                 continue;

    //             arr.insert(make_pair(j, statements[i][j]));
    //         }

    //         for (int j = 0; j < statements[i].size(); ++j) {
    //             if (runed[j])
    //                 continue;

    //             auto it = arr.find(j);
    //             if (it != arr.end()) {
    //                 do { // 假设 i 是好人
    //                     if (statements[i][j] != 2 && it->second != statements[i][j])
    //                         return; // 不符合逻辑

    //                     tmp_max += it->second;
    //                     runed[j] = 1;
    //                     dfs(j, it->second);
    //                     runed[j] = 0;
    //                     tmp_max -= it->second;
    //                 } while (0);

    //                 // i 是坏人
    //                 tmp_max += it->second;
    //                 runed[j] = 1;
    //                 dfs(j, it->second);
    //                 runed[j] = 0;
    //                 tmp_max -= it->second;
    //             } else {
    //                 fun(j, arr, statements, runed, dfs);
    //             }
    //         }
    //         runed[i] = 0;
    //         arr.swap(tmp_map);
    //     };

public:
    // 不能贪心吗? 不能 因为都说坏人的情况下, 至少会有1个好人, 因为求的是好人的最大数量
    int maximumGood(vector<vector<int>>& statements) { // 0坏 1好 2略
        int res = 0;
        int tmp_max = 0;

        vector<bool> runed(statements.size());
        // 假设是i好人 / 假设是坏人
        map<int, int> arr;  // 人 - 当前属性
                            // 如果不符逻辑就return回溯
        function<void(int, int)> dfs =
        [&](int i, int tag) {
            if ( findVector(runed) ) {
                res = max(tmp_max, res);
                return;
            }
            
            for (int j = 0; j < statements[i].size(); ++j) {
                if (runed[j])
                    continue;

                auto it = arr.find(j);
                if (it != arr.end()) {
                    if (tag && statements[i][j] != 2 && it->second != statements[i][j])
                        return; // 不符合逻辑

                    tmp_max += it->second;
                    runed[j] = 1;
                    dfs(j, it->second);
                    runed[j] = 0;
                    tmp_max -= it->second;
                } else {
                    // fun(j, arr, statements, runed, dfs);
                    runed[j] = 1;
                    tmp_max += 1;
                    arr.insert(make_pair(j, 1));
                    dfs(j, 1);
                    arr.erase(j);
                    tmp_max -= 1;
                    dfs(j, 0);
                    runed[j] = 0;
                    return;
                }
            }
            
        };

        [&]() {
            for (int i = 0; i < statements.size(); ++i) {
                arr = map<int, int>();
                arr.insert(make_pair(i, 1)); // i 是好人
                runed[i] = 1;
                for (int j = 0; j < statements[i].size(); ++j) {
                    if (statements[i][j] == 2)
                        continue;

                    arr.insert(make_pair(j, statements[i][j]));
                }

                for (int j = 0; j < statements[i].size(); ++j) {
                    if (runed[j])
                        continue;

                    tmp_max += arr.find(j)->second;
                    dfs(j, arr.find(j)->second);
                    tmp_max -= arr.find(j)->second;
                }
                runed[i] = 0;
            }
        }();

        return res;
    }
};
```



```C++
class Solution {
    bool findVector(vector<bool>& runed) {
        int n = runed.size();
        for (int i = 0; i < n; ++i)
            if (!runed[i])
                return false;
        return true;
    }
// [[2,2,2],[2,2,2],[2,2,2]]
public:
    // 不能贪心吗? 不能 因为都说坏人的情况下, 至少会有1个好人, 因为求的是好人的最大数量
    int maximumGood(vector<vector<int>>& statements) { // 0坏 1好 2略
        int res = 0;
        int tmp_max = 0;

        vector<bool> runed(statements.size());
        // 假设是i好人 / 假设是坏人
        map<int, int> arr;  // 人 - 当前属性
                            // 如果不符逻辑就return回溯
        function<void(int, int)> dfs =
        [&](int i, int tag) {
            function<void(int)> fun =
            [&](int j) {
                if (j >= (int)statements.size())
                    return;
                printf(">>> j: %d |  %d\n", j, tmp_max);
                // 假设 j 是 好人
                map<int, int> tmp_map = arr;
                for (int k = 0; k < statements[j].size(); ++k) {
                    if (statements[j][k] == 2)
                        continue;
                    
                    auto it = arr.find(k);
                    if (it != arr.end() && it->second != statements[j][k])
                        goto B; // 不符合逻辑

                    arr.insert(make_pair(j, statements[j][k]));         
                }

                for (int k = 0; k < statements[j].size(); ++k) {
                    if (runed[k])
                        continue;

                    auto it = arr.find(k);
                    if (it != arr.end()) {
                        runed[k] = 1;
                        tmp_max += it->second;
                        dfs(k, it->second);
                        tmp_max -= it->second;
                        runed[k] = 0;
                    }
                    else {
                        runed[k] = 1;
                        fun(k);
                        runed[k] = 0;
                    }
                }

                B:
                arr.swap(tmp_map);
                for (int k = 0; k < statements[j].size(); ++k) {
                    if (runed[k])
                        continue;
                    
                    runed[k] = 1;
                    dfs(k, 0);
                    runed[k] = 0;
                }
                printf("j: %d |  %d\n", j, tmp_max);
            };
////////////////////////////////////////////////////////////////////
            if ( findVector(runed) ) {
                res = max(tmp_max, res);
                return;
            }
            printf("i: %d  |  %d\n", i, tmp_max);
            for (int j = 0; j < statements[i].size(); ++j) {
                if (runed[j])
                    continue;

                auto it = arr.find(j);
                if (it != arr.end()) {
                    if (tag && statements[i][j] != 2 && it->second != statements[i][j])
                        return; // 不符合逻辑

                    tmp_max += it->second;
                    runed[j] = 1;
                    printf("++++");
                    int op = it->second;
                    dfs(j, op);
                    runed[j] = 0;
                    printf("HHHH");
                    tmp_max -= op;
                    printf("$$$$");
                }
                else {
                    runed[j] = 1;
                    fun(j);
                    runed[j] = 0;
                }
            }

            printf("///");
        };

        [&]() {
            for (int i = 0; i < statements.size(); ++i) {
                arr = map<int, int>();
                arr.insert(make_pair(i, 1)); // i 是好人
                runed[i] = 1;
                tmp_max += 1;
                for (int j = 0; j < statements[i].size(); ++j) {
                    if (statements[i][j] == 2)
                        continue;

                    arr.insert(make_pair(j, statements[i][j]));
                }

                for (int j = 0; j < statements[i].size(); ++j) {
                    if (runed[j])
                        continue;

                    printf("----");
                    tmp_max += arr.find(j)->second;
                    dfs(j, arr.find(j)->second);
                    tmp_max -= arr.find(j)->second;
                    printf("!!!");
                }
                tmp_max -= 1;
                runed[i] = 0;
            }
        }();

        return res;
    }
};
```

就差该死的 2 的情况!
```C++
class Solution {
    bool findVector(vector<bool>& runed) {
        int n = runed.size();
        for (int i = 0; i < n; ++i)
            if (!runed[i])
                return false;
        return true;
    }
// [[2,2,2],[2,2,2],[2,2,2]]
public:
    // 不能贪心吗? 不能 因为都说坏人的情况下, 至少会有1个好人, 因为求的是好人的最大数量
    int maximumGood(vector<vector<int>>& statements) { // 0坏 1好 2略
        int res = 0;
        int tmp_max = 0;

        vector<bool> runed(statements.size());
        // 假设是i好人 / 假设是坏人
        map<int, int> arr;  // 人 - 当前属性
                            // 如果不符逻辑就return回溯
        function<void(int, int)> dfs =
        [&](int i, int tag) {
            function<void(int)> fun =
            [&](int j) {
                
                printf(">>> j: %d |  %d\n", j, tmp_max);
                // 假设 j 是 好人
                map<int, int> tmp_map = arr;
                arr.insert(make_pair(j, 1)); // j 是好人
                tmp_max += 1;
                for (int k = 0; k < statements[j].size(); ++k) {
                    if (statements[j][k] == 2)
                        continue;
                    
                    auto it = arr.find(k);
                    if (it != arr.end() && it->second != statements[j][k])
                        goto B; // 不符合逻辑

                    arr.insert(make_pair(k, statements[j][k]));         
                }

                for (int k = 0; k < statements[j].size(); ++k) {
                    if (runed[k])
                        continue;

                    auto it = arr.find(k);
                    if (it != arr.end()) {
                        runed[k] = 1;
                        tmp_max += it->second;
                        dfs(k, it->second);
                        tmp_max -= it->second;
                        runed[k] = 0;
                    }
                    else {
                        runed[k] = 1;
                        fun(k);
                        runed[k] = 0;
                    }
                }

                B: // 坏人
                tmp_max -= 1;
                arr.swap(tmp_map);
                for (int k = 0; k < statements[j].size(); ++k) {
                    if (runed[k])
                        continue;
                    
                    runed[k] = 1;
                    dfs(k, 0);
                    runed[k] = 0;
                }
                printf("j: %d |  %d\n", j, tmp_max);
            };
////////////////////////////////////////////////////////////////////
            if ( findVector(runed) ) {
                res = max(tmp_max, res);
                return;
            }
            printf("i: %d  |  %d\n", i, tmp_max);
            for (int j = 0; j < statements[i].size(); ++j) {
                if (runed[j])
                    continue;

                auto it = arr.find(j);
                if (it != arr.end()) {
                    if (tag && statements[i][j] != 2 && it->second != statements[i][j])
                        return; // 不符合逻辑

                    tmp_max += it->second;
                    runed[j] = 1;
                    // printf("++++");
                    int op = it->second;
                    dfs(j, op);
                    runed[j] = 0;
                    // printf("HHHH");
                    tmp_max -= op;
                    // printf("$$$$");
                }
                else {
                    runed[j] = 1;
                    fun(j);
                    runed[j] = 0;
                }
            }

            // printf("///");
        };

        [&]() {
            for (int i = 0; i < statements.size(); ++i) {
                arr = map<int, int>();
                arr.insert(make_pair(i, 1)); // i 是好人
                runed[i] = 1;
                tmp_max += 1;
                for (int j = 0; j < statements[i].size(); ++j) {
                    if (statements[i][j] == 2)
                        continue;

                    arr.insert(make_pair(j, statements[i][j]));
                }

                for (int j = 0; j < statements[i].size(); ++j) {
                    if (runed[j])
                        continue;

                    // printf("----");
                    tmp_max += arr.find(j)->second;
                    dfs(j, arr.find(j)->second);
                    tmp_max -= arr.find(j)->second;
                    // printf("!!!");
                }
                tmp_max -= 1;
                runed[i] = 0;
            }
        }();

        return res;
    }
};
```


```C++
class Solution {
    bool findVector(vector<bool>& runed) {
        for (int i = 0; i < runed.size(); ++i)
            if (!runed[i])
                return false;
        return true;
    }
public:
    int maximumGood(vector<vector<int>>& statements) {
        vector<int> arr;
        vector<bool> runed(statements.size());
        int res = 0;
        int tmp_max = 0;

        function<void(int, int)> dfs =
        [&](int i, int tag) {
            if ( findVector(runed) ) {
                // res = max(res, tmp_max);
                int _tmp = 0;
                for (int& it : arr)
                    if (it == 1)
                        ++_tmp;
                res = max(res, _tmp);
                return;
            }

            if (tag == 1 || tag == 2) { // 好人
                for (int j = 0; j < statements[i].size(); ++j) {
                    if (runed[j] || i == j)
                        continue;
                    
                    // 不是未知, 且 观点不符合逻辑
                    if (statements[i][j] != 2 && arr[j] != statements[i][j]) {
                        return;
                    }

                    runed[j] = true;
                    // int tmp = arr[j];
                    // arr[j] = (statements[i][j] == 2 ? 1 : statements[i][j]);
                    dfs(j, (statements[i][j] == 2 ? 1 : statements[i][j]));
                    // arr[j] = tmp;
                    runed[j] = false;
                }
            }

            if (tag == 0 || tag == 2) { // 坏人
                return;
            }
        };

        for (int i = 0; i < statements.size(); ++i) {
            arr = vector<int>(statements.size());
            for (int j = 0; j < statements[i].size(); ++j)
                arr[j] = statements[i][j];
            arr[i] = 1;
            
            runed[i] = true;
            dfs(i, 1);
            runed[i] = false;
        }

        return res;
    }
};
```
