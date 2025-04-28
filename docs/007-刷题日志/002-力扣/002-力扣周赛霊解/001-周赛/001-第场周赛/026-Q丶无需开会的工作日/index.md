# 3169. 无需开会的工作日
链接: [3169. 无需开会的工作日](https://leetcode.cn/problems/count-days-without-meetings/)

给你一个正整数 days，表示员工可工作的总天数（从第 1 天开始）。另给你一个二维数组 meetings，长度为 n，其中 meetings[i] = [start_i, end_i] 表示第 i 次会议的开始和结束天数（包含首尾）。

返回员工可工作且没有安排会议的天数。

注意：会议时间可能会有重叠。

# 题解

同: [56. 合并区间](https://leetcode.cn/problems/merge-intervals/)

## 合并区间

按区间左端点排序后...

```C++
class Solution {
public:
    int countDays(int days, vector<vector<int>>& meetings) {
        sort(meetings.begin(), meetings.end(), 
             [](const auto& a, const auto& b){
                 return a[0] < b[0];
             });
        
        // 合并区间
        vector<vector<int>> qj;
        qj.push_back(meetings[0]);
        int sum = 0, tag = 1;
        for (int i = 1; i < meetings.size(); ++i) {
            if (meetings[i][0] <= qj[qj.size() - 1][1]) {
                qj[qj.size() - 1][1] = max(qj[qj.size() - 1][1], meetings[i][1]);
            } else {
                sum += qj[qj.size() - 1][1] - qj[qj.size() - 1][0] + 1;
                qj.push_back(meetings[i]);
            }
        }
        
        sum += qj[qj.size() - 1][1] - qj[qj.size() - 1][0] + 1;
        
        return days - sum;
    }
};
```

## 0x3f 简洁写法

```C++
class Solution {
public:
    int countDays(int days, vector<vector<int>>& meetings) {
        ranges::sort(meetings); // 按照左端点从小到大排序
        int start = 1, end = 0; // 当前合并区间的左右端点
        for (auto& p : meetings) {
            if (p[0] > end) { // 不相交
                days -= end - start + 1; // 当前合并区间的长度
                start = p[0]; // 下一个合并区间的左端点
            }
            end = max(end, p[1]);
        }
        days -= end - start + 1; // 最后一个合并区间的长度
        return days;
    }
};
```
