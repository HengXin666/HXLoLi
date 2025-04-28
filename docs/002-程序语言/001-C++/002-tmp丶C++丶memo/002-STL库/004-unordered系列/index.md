# unordered系列
## 说明
C++11 引入了两组无序容器：

`std::unordered_set`不允许重复的集合 / `std::unordered_multiset`允许重复的集合

`std::unordered_map`不允许重复的字典 / `std::unordered_multimap`允许重复的字典

**无序容器中的元素是不进行排序的**，内部通过` Hash 表`实现，插入和搜索元素的平均复杂度为 O(1). 

头文件:
```C++
#include <unordered_map>
#include <unordered_set>
```

## 哈希表的性能注意
如果单纯的查询元素, 而不增加它的话, 建议使用方法`.find(key) != .end()`或者`.count(key)`判断是否存在, 如果存在才进行`hash[key]`! 因为如果直接`hash[key]`的话, 那么相当于先`.insert(key)`, 会有一个插入的时间, 可能会引发 *哈希碰撞/哈希表扩容* 导致时间被無駄ら!

## 使用
### unordered_map
`unordered_map`是一个无序键值对的容器，其中的元素根据其键进行快速查找。它使用哈希函数来映射键到索引位置，从而实现快速访问
```C++
std::unordered_map<KeyType, ValueType> map;

// 插入键值对
map.insert(std::make_pair(key, value));

// 通过键来访问值
ValueType value = map[key];
```

### unordered_set
`unordered_set`是一个无序集合容器，存储不重复的元素。它使用哈希函数来确定元素的存储位置，从而实现快速查找。
```C++
std::unordered_set<ValueType> set;

// 插入元素
set.insert(value);

// 查找元素
bool found = set.find(value) != set.end();
```

### 哈希函数
unordered容器使用哈希函数来将键或元素映射到索引位置。在自定义类型作为键或元素时，你需要提供自己的哈希函数

```C++
struct MyClassHash {
    std::size_t operator()(const MyClass& obj) const {
        // 返回哈希值
    }
};

// 创建unordered容器时指定哈希函数
std::unordered_map<MyClass, ValueType, MyClassHash> map;
std::unordered_set<MyClass, MyClassHash> set;
```

### 性能注意事项
- unordered容器提供了快速的插入、查找和删除操作。
- 使用哈希函数来映射元素到索引位置，因此要确保哈希函数能够产生均匀分布的值，以避免冲突和性能下降。
- 默认情况下，unordered容器使用`std::hash`作为哈希函数。对于自定义类型，你需要自己提供哈希函数。

### 内部链接

#### [1]

使用实例！刷题不要局限字符串是单字符组成了！现在你可以哈希它！

[2707. 字符串中的额外字符](../../../../../007-刷题日志/002-力扣/003-未分类题解/029-字符串中的额外字符/index.md)