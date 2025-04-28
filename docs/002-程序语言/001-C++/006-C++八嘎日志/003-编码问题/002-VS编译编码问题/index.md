# 字节已用Unicode替换字符替换

- [VS异常：文件乱码：文件加载，使用Unicode(UTF-8)编码加载文件xxx时，有些字节已用Unicode替换字符替换。保存该文件将不会保留原始文件内容。](https://blog.csdn.net/bugang4663/article/details/110408633) <-- 只看现象, 解决方案先不看

尝试使用: 

## utf-8 编译

有三种办法, 你看看那种可以用就用那种.

### 1. CMake
- `CMakeLists.txt`里面加个`add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")`以utf-8编译

### 2. 项目属性

`项目 > 属性 > c/c++ > 命令行 > 加上 \utf-8`

### 3. 修改.json

在`CMakeSettings.json`里面配置(`x64-Debug的下拉菜单 > 管理配置 > 编辑json`)
```json
"variables": [
    {
        "name": "CMAKE_CXX_FLAGS",
        "value": "/source-charset:utf-8"
    }
]
```

示例:
```json
{
    "configurations": [
        {
            "name": "x64-Debug",
            "generator": "Ninja",
            "configurationType": "Debug",
            "buildRoot": "${projectDir}\\out\\build\\${name}",
            "installRoot": "${projectDir}\\out\\install\\${name}",
            "cmakeCommandArgs": "",
            "ctestCommandArgs": "",
            "inheritEnvironments": [ "msvc_x64_x64" ],
            "variables": [
                {
                    "name": "CMAKE_CXX_FLAGS",
                    "value": "/source-charset:utf-8"
                }
            ]
        }
    ]
}
```