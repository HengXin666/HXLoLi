# 二分查找
> 学习: [二分查找 红蓝染色法](https://www.bilibili.com/video/BV1AP41137w7/)

## 三种写法
### 闭区间 [L, R]

```C++
// lower_bound 返回最小的满足 nums[i] >= target 的 i
// 如果数组为空，或者所有数都 < target，则返回 nums.size()
// 要求 nums 是非递减的，即 nums[i] <= nums[i + 1]

// 闭区间写法
int lower_bound(vector<int> &nums, int target) {
    int left = 0, right = nums.size() - 1; // 闭区间 [left, right]
    while (left <= right) { // 区间不为空
        // 循环不变量:
        // nums[left-1] < target
        // nums[right+1] >= target
        int mid = left + (right - left) / 2;
        if (nums[mid] < target) {
            left = mid + 1; // 范围缩小到 [mid+1, right]
        } else {
            right = mid - 1; // 范围缩小到 [left, mid-1]
        }
    }
    return left;
}

// 作者:灵茶山艾府
// 链接:https://leetcode.cn/problems/find-first-and-last-position-of-element-in-sorted-array/solutions/1980196/er-fen-cha-zhao-zong-shi-xie-bu-dui-yi-g-t9l9/
// 来源:力扣（LeetCode）
// 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```

### 左闭右开 [L, R)

```C++
// 左闭右开区间写法
int lower_bound(vector<int> &nums, int target) {
    int left = 0, right = nums.size(); // 左闭右开区间 [left, right)
    while (left < right) { // 区间不为空
        // 循环不变量:
        // nums[left-1] < target
        // nums[right] >= target
        int mid = left + (right - left) / 2;
        if (nums[mid] < target) {
            left = mid + 1; // 范围缩小到 [mid+1, right)
        } else {
            right = mid; // 范围缩小到 [left, mid)
        }
    }
    return left; // 返回 left 还是 right 都行，因为循环结束后 left == right
}

// 作者:灵茶山艾府
// 链接:https://leetcode.cn/problems/find-first-and-last-position-of-element-in-sorted-array/solutions/1980196/er-fen-cha-zhao-zong-shi-xie-bu-dui-yi-g-t9l9/
// 来源:力扣（LeetCode）
// 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```

### 开区间 (L, R)

```C++
// 开区间写法
int lower_bound(vector<int> &nums, int target) {
    int left = -1, right = nums.size(); // 开区间 (left, right)
    while (left + 1 < right) { // 区间不为空
        // 循环不变量:
        // nums[left] < target
        // nums[right] >= target
        int mid = left + (right - left) / 2;
        if (nums[mid] < target) {
            left = mid; // 范围缩小到 (mid, right)
        } else {
            right = mid; // 范围缩小到 (left, mid)
        }
        // 也可以这样写
        // (nums[mid] < target ? left : right) = mid;
    }
    return right;
}

// 作者:灵茶山艾府
// 链接:https://leetcode.cn/problems/find-first-and-last-position-of-element-in-sorted-array/solutions/1980196/er-fen-cha-zhao-zong-shi-xie-bu-dui-yi-g-t9l9/
// 来源:力扣（LeetCode）
// 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```


## 转化: 四种类型
- 有序数组中二分查找的四种类型（下面的转换仅适用于数组中都是整数）
    1. 第一个大于等于 $x$ 的下标: $low\_bound(x)$
    
    2. 第一个大于 $x$ 的下标: 可以转换为`第一个大于等于 x + 1 的下标`, $low\_bound(x+1)$
    
    3. 最后一个小于 $x$ 的下标: 可以转换为`第一个大于等于 x 的下标` 的`左边位置`, $low\_bound(x) - 1$
    
    4. 最后一个小于等于 $x$ 的下标: 可以转换为`第一个大于等于 x + 1 的下标` 的 `左边位置`, $low\_bound(x+1) - 1$