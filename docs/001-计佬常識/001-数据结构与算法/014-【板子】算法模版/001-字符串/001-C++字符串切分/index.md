# C++字符串切分

选自我的项目的工具函数: [HXSTL/utils/StringUtils.h](https://github.com/HengXin666/HXLibs/blob/main/include/HXSTL/utils/StringUtils.h) (里面还有很多)

### 1. split

```cpp
/**
 * @brief 将字符串按`delimiter`分割为数组
 * @tparam Str 字符串类型, 可以是`std::string`或者`std::string_view`等
 * @tparam SkipEmpty 是否跳过切割出来的空的字符串
 * @param str 需要分割的字符串
 * @param delim 分割字符串
 * @param res 待返回数组, 你可以事先在前面插入元素
 * @return std::vector<Str> 
 */
template <typename Str, bool SkipEmpty = true>
inline std::vector<Str> split(
    std::string_view str,
    std::string_view delim, 
    std::vector<Str> res = std::vector<Str>{}
) {
    if (str.empty()) 
        return res;

    std::size_t start = 0;
    std::size_t end = str.find(delim, start);
    while (end != std::string_view::npos) {
        if constexpr (SkipEmpty) {
            auto tk = str.substr(start, end - start);
            if (tk.size()) {
                res.emplace_back(std::move(tk));
            }
        } else {
            res.emplace_back(str.substr(start, end - start));
        }
        start = end + delim.size();
        end = str.find(delim, start);
    }

    // 添加最后一个分割的部分
    if constexpr (SkipEmpty) {
        auto tk = str.substr(start);
        if (tk.size()) {
            res.emplace_back(std::move(tk));
        }
    } else {
        res.emplace_back(str.substr(start));
    }
    return res;
}
```

- 使用示例:

```cpp
string path = "/home/../loli/./.../////a/b/c///";
auto strList = split<string_view>(path, "/");

// 得到: ["home", "..", "loli", ".", "...", "a", "b", "c"]
```