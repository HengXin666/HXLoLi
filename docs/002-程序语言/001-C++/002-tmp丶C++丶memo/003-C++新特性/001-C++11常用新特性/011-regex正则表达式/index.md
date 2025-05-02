# std::regex

C++ 的正则表达式 API 提供在 `<regex>` 头文件中，常用的正则表达式处理库包括以下几个类与函数：

## 1. `std::regex`
- **功能**：用于表示正则表达式的对象。
- **构造函数**：
  - `regex()`: 默认构造函数，构造一个空的正则表达式对象。
  - `regex(const std::string&)`: 根据传入的字符串来构造正则表达式对象。

**示例**：
```cpp
#include <regex>
#include <iostream>

int main() {
    std::regex pattern("\\d+"); // 匹配数字
    std::string text = "There are 100 apples.";

    if (std::regex_search(text, pattern)) {
        std::cout << "Found a number!" << std::endl;
    }
    return 0;
}
```

**底层实现**：
- `std::regex` 通常通过 DFA（确定有限状态自动机）或 NFA（不确定有限状态自动机）来匹配字符串，这取决于具体实现。`std::regex` 的标准库实现可能会优化模式编译和匹配过程，但在复杂的表达式下，性能可能不及一些专用的正则库（如 PCRE）。

**性能分析**：
- `std::regex` 会在使用时解析正则表达式模式，并生成一个匹配引擎。这个解析过程可能比较耗时，尤其对于复杂的正则表达式而言。在大量正则操作的场景下，建议避免频繁创建 `std::regex` 对象，尽量重用同一个对象。

## 2. `std::regex_search`
- **功能**：在给定文本中搜索是否存在符合正则表达式的子串。
- **函数原型**：
  - `bool regex_search(const std::string& s, const std::regex& rgx)`: 检查是否有匹配项。
  - `bool regex_search(const std::string& s, std::smatch& match, const std::regex& rgx)`: 获取匹配内容。

**示例**：
```cpp
#include <regex>
#include <iostream>

int main() {
    std::regex pattern("\\d+");
    std::string text = "There are 100 apples.";
    std::smatch match;

    if (std::regex_search(text, match, pattern)) {
        std::cout << "Found number: " << match.str() << std::endl;
    }
    return 0;
}
```

**底层实现**：
- `regex_search` 实现了字符串的部分匹配，通常会从字符串的起点依次推进来寻找匹配。如果找到满足的子串，它将终止搜索并返回成功。

**性能分析**：
- 部分匹配通常比全匹配快，因为不需要匹配整个字符串。对于较长的文本，`regex_search` 的性能受限于字符串的长度和正则表达式的复杂度。
  
## 3. `std::regex_match`
- **功能**：尝试匹配整个字符串。
- **函数原型**：
  - `bool regex_match(const std::string& s, const std::regex& rgx)`: 检查整个字符串是否匹配。
  - `bool regex_match(const std::string& s, std::smatch& match, const std::regex& rgx)`: 获取完整匹配的内容。

**示例**：
```cpp
#include <regex>
#include <iostream>

int main() {
    std::regex pattern("^\\d+$"); // 匹配完整的数字字符串
    std::string text = "12345";

    if (std::regex_match(text, pattern)) {
        std::cout << "The whole string is a number!" << std::endl;
    }
    return 0;
}
```

**底层实现**：
- `regex_match` 使用 DFA 或 NFA 来处理完整的字符串匹配。与 `regex_search` 不同的是，`regex_match` 需要检查整个字符串，因此对性能有更高的要求。

**性能分析**：
- `regex_match` 的性能通常较 `regex_search` 要差，尤其在长字符串和复杂表达式的情况下。它必须验证整个字符串是否满足正则表达式的要求，计算开销较高。

## 4. `std::regex_replace`
- **功能**：替换匹配的子串。
- **函数原型**：
  - `std::string regex_replace(const std::string& s, const std::regex& rgx, const std::string& format)`: 用给定的格式替换匹配的部分。

**示例**：
```cpp
#include <regex>
#include <iostream>

int main() {
    std::regex pattern("\\d+"); // 匹配数字
    std::string text = "I have 100 apples and 200 oranges.";

    std::string result = std::regex_replace(text, pattern, "XXX");
    std::cout << result << std::endl; // 输出：I have XXX apples and XXX oranges.
    return 0;
}
```

**底层实现**：
- `regex_replace` 会在找到每一个匹配时进行替换操作，效率取决于匹配的数量和替换操作的复杂性。它基于匹配器在原字符串上逐步推进，执行替换。

**性能分析**：
- 替换操作的性能主要受到匹配项的数量影响。替换字符串的操作本身较快，但正则表达式匹配的性能瓶颈依旧是模式解析和匹配过程。尽可能避免使用过于复杂的正则模式。

## 5. `std::smatch` 和 `std::cmatch`
- **功能**：`std::smatch` 是用于保存匹配结果的类，适用于 `std::string`；`std::cmatch` 适用于 C 风格字符串。
- **常用成员函数**：
  - `str()`: 获取匹配的子串。
  - `position()`: 获取匹配子串的起始位置。
  - `length()`: 获取匹配子串的长度。

**示例**：
```cpp
#include <regex>
#include <iostream>

int main() {
    std::regex pattern("(\\d+)");
    std::string text = "Price: 250";

    std::smatch match;
    if (std::regex_search(text, match, pattern)) {
        std::cout << "Matched: " << match.str(1) << std::endl;
        std::cout << "Position: " << match.position(1) << std::endl;
        std::cout << "Length: " << match.length(1) << std::endl;
    }
    return 0;
}
```

**底层实现**：
- `smatch` 和 `cmatch` 类背后实现了一个匹配结果的容器，存储每一次匹配的开始和结束位置，以及匹配的子串。每次正则匹配成功后，都会将匹配的结果填充到这个容器中。

**性能分析**：
- 匹配结果的存储操作在大部分情况下开销较小，但大量匹配结果的存储可能会对内存使用产生影响。在性能敏感的场合，可以避免不必要的存储操作。

### 总结
C++ 的正则表达式库虽然功能强大，但其性能相对于一些专门的正则库（如 PCRE、RE2 等）较慢，特别是在处理复杂表达式或大量匹配时。C++ 标准库中的正则表达式主要适合中小规模的文本处理任务。