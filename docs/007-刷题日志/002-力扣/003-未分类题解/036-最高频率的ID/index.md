# 3092. 最高频率的 ID

原题: [3092. 最高频率的 ID](https://leetcode.cn/problems/most-frequent-ids/description/)

390周赛 Q3 中等

你需要在一个集合里动态记录 ID 的出现频率。给你两个长度都为 n 的整数数组 nums 和 freq ，nums 中每一个元素表示一个 ID ，对应的 freq 中的元素表示这个 ID 在集合中此次操作后需要增加或者减少的数目。

- 增加 ID 的数目：如果 freq[i] 是正数，那么 freq[i] 个 ID 为 nums[i] 的元素在第 i 步操作后会添加到集合中。
- 减少 ID 的数目：如果 freq[i] 是负数，那么 -freq[i] 个 ID 为 nums[i] 的元素在第 i 步操作后会从集合中删除。

请你返回一个长度为 n 的数组 ans ，其中 ans[i] 表示第 i 步操作后出现频率最高的 ID 数目 ，如果在某次操作后集合为空，那么 ans[i] 为 0 。

 
```
示例 1：

输入：nums = [2,3,2,1], freq = [3,2,-3,1]

输出：[3,3,2,2]

解释：

第 0 步操作后，有 3 个 ID 为 2 的元素，所以 ans[0] = 3 。
第 1 步操作后，有 3 个 ID 为 2 的元素和 2 个 ID 为 3 的元素，所以 ans[1] = 3 。
第 2 步操作后，有 2 个 ID 为 3 的元素，所以 ans[2] = 2 。
第 3 步操作后，有 2 个 ID 为 3 的元素和 1 个 ID 为 1 的元素，所以 ans[3] = 2 。

示例 2：

输入：nums = [5,5,3], freq = [2,-2,1]

输出：[2,0,1]

解释：

第 0 步操作后，有 2 个 ID 为 5 的元素，所以 ans[0] = 2 。
第 1 步操作后，集合中没有任何元素，所以 ans[1] = 0 。
第 2 步操作后，有 1 个 ID 为 3 的元素，所以 ans[2] = 1 。
```
 

提示：

$
1 <= nums.length == freq.length <= 10^5\\
1 <= nums[i] <= 10^5\\
-10^5 <= freq[i] <= 10^5\\
freq[i] != 0\\
输入保证任何操作后，集合中的元素出现次数不会为负数。\\
$

# 题解
## AC
我的代码: 类似于 [2671. 频率跟踪器](https://leetcode.cn/problems/frequency-tracker/description/) 这题

使用倆哈希表, 注意数量可以重复的, 所以第二个要使用`multimap` (实际上使用`multiset`也可以)
```C++
class Solution {
public:
    vector<long long> mostFrequentIDs(
        vector<int>& nums,  // id
        vector<int>& f      // 个数
    ) {
/*
id - 数量
数量 - id

问最大数量
*/
        int n = nums.size();
        vector<long long> res(n);

        unordered_map<int, long long> id;
        multimap<long long, int> m;
        for (int i = 0; i < n; ++i) {
            auto it = m.find(id[nums[i]]);
            for (; it != m.end() && it->second == nums[i]; ++it) {
                if (it->first == id[nums[i]])
                    break;
            }
            if (it != m.end())
                m.erase(it);
            id[nums[i]] += f[i];
            m.insert({id[nums[i]], nums[i]}); // 似乎不能 直接key插入访问?

            res[i] = m.rbegin()->first; // 使用反向迭代器! 得最大值
        }

        return res;
    }
};
```

## 0x3f的 懒删除堆

这个很简单!
> 0x3f say:
>
> 在堆中查询 $\textit{cnt}[x]$ 的最大值时，如果堆顶保存的数据并不是目前实际的 $\textit{cnt}[x]$，那么就弹出堆顶。

```C++
class Solution {
public:
    vector<long long> mostFrequentIDs(vector<int> &nums, vector<int> &freq) {
        int n = nums.size();
        vector<long long> ans(n);
        unordered_map<int, long long> cnt;
        priority_queue<pair<long long, int>> pq;
        for (int i = 0; i < n; i++) {
            int x = nums[i];
            cnt[x] += freq[i];
            pq.emplace(cnt[x], x);
            while (pq.top().first != cnt[pq.top().second]) { // 堆顶保存的数据已经发生变化
                pq.pop(); // 删除
            }
            ans[i] = pq.top().first;
        }
        return ans;
    }
};

// 作者：灵茶山艾府
// 链接：https://leetcode.cn/problems/most-frequent-ids/solutions/2704858/ha-xi-biao-you-xu-ji-he-pythonjavacgo-by-7brw/
// 来源：力扣（LeetCode）
// 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```

我的实现

```C++
class Solution {
public:
    vector<long long> mostFrequentIDs(
        vector<int>& nums, vector<int>& freq) {
        // 使用懒标记大堆 <数量, id>
        // 因为需要更新 数量, 但是不可能总是去更新
        // 我们需要获取的是数量的最大值
        // 所以 可以使用一个大根堆
        // 采用懒更新, 如果取的top的 数量 != id[id] 那么 显然已经被更新了
        // 此时就再入堆, 再取
        vector<long long> res(nums.size());
        unordered_map<int, long long> id;        // id - 数量
        priority_queue<pair<long long, int>> pq; // 默认大根堆
        for (int i = 0; i < nums.size(); ++i) {
            id[nums[i]] += freq[i]; // 更新数量
            pq.push({id[nums[i]], nums[i]});
            auto it = pq.top();
            while (it.first != id[it.second]) {
                pq.pop();
                pq.push({id[it.second], it.second}); // 更新
                it = pq.top();
            }
            res[i] = it.first;
        }
        return res;
    }
};
```
