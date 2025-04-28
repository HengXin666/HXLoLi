# w_char 与 char 类型转换
[C++ WINDOWS下 wchar_t *和char * 相互转化总结篇](https://blog.csdn.net/lightspear/article/details/54695123)

```C++
// 类型转换 string --> wstring
inline std::wstring to_wide_string(const std::string& input)
{
    std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
    return converter.from_bytes(input);
}

// 类型转换 wstring --> string 
inline std::string to_byte_string(const std::wstring& input)
{
    //std::wstring_convert<std::codecvt_utf8_utf16<wchar_t>> converter;
    std::wstring_convert<std::codecvt_utf8<wchar_t>> converter;
    return converter.to_bytes(input);
}
```
