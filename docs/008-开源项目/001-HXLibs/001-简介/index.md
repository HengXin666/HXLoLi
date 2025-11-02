# HXLibs
## 一、简介

HXLibs 是一个现代C++基础工具库, 目前包括以下模块:

- container (容器库)
    - 编译期容器: 编译期 vector / 编译期完美哈希表
    - void 类型擦除类型 / 未初始化容器 / 未初始化且擦除void的 Variant 容器
    - 线程安全容器: SafeQueue
    - 擦除void的 Try 容器 (可存放数据或异常指针)
    - 线程池 / 未来体 `FutureResult` (适配 线程池 & 协程)
    - *一些内部使用的: MoveOnlyFunction.hpp / MoveApply.hpp*
- coroutine (协程库)
    - awaiter (包含可被 `co_await` 的对象, 包括 WhenAll / WhenAny)
    - concepts (协程概念)
    - loop (协程事件循环, 包括 基于IOCP/io_uring的操作系统的事件循环、定时器循环、线程池循环)
    - promise
    - task (协程, 包括 普通协程、根协程、异步IO协程)
- exception (异常库)
    - 定义了一些枚举和操作系统API诊断. 待更新异常类
- log (日志库)
    - 支持打印 STL 容器、聚合类、带缩进格式打印
    - 可注册打印类型
    - 目前仅支持打印. 文件日志写入待支持
- macro (宏库)
    - 警告屏蔽宏
    - For 展开宏、连接宏 等
- meta (元编程库)
    - 容器概念
    - 可作为 NTTP 的字符串
    - 函数参数萃取模板
    - 哈希
    - 类型特征
- net (网络库)
    - Http(s) 客户端 / 服务端
        - 服务端: http(s), websocket, 响应: 分块编码 / 断点续传
        - 客户端: socks5 / http 代理, http(s), websocket, 解析: 分块编码 / 断点续传
    - 客户端池: 可自定义负载均衡算法, 使用多个客户端实例
    - API宏: 方便用户快速定义 控制器端点, 并支持依赖注入
- platform (内部使用: 平台相关内容)
- reflection (反射库)
    - 枚举反射
    - 类名反射
    - 聚合类反射: 反射成员个数、反射成员名称、反射: `<成员指针, 成员名称>编译期哈希表` / `<成员名称, 类字段偏移量>编译期哈希表`
    - 宏反射: 支持全局、类内部对类成员进行反射注册, 支持类成员定义别名
    - json 基于反射的序列化 & 反序列化库
- utils (工具库)
    - 大端小端编译期判断 / 大端小端转换
    - 文件信息, 读写
    - 进制转化工具类
    - 编译期随机数 (XorShift32)
    - 字符串操作工具类 / 时间格式工具类
    - RAII 计时器工具, 支持计算循环的
    - NTTP Time类型, 可转为 STL 的 Chrono 类型

## 二、快速开始
### 2.1 编译器要求

最低C++标准为: C++20

- G++ 14 或更高版本
- Clang++ 18 或更高版本
- msvc 14.29 或更高版本

> [!TIP]
> 没有测试过更低的, 支持 C++20 的编译器.

### 2.2 安装 & 编译
#### 2.2.1 通过vcpkg包管理器安装

> [!TIP]
> 待支持, 期望支持

```cmake
find_package(HXLibs CONFIG REQUIRED)
target_link_libraries(YouProject PRIVATE HXLibs::HXLibs)
```

#### 2.2.2 Cmake FetchContent

> [!TIP]
> 待支持, 期望支持

```cmake
include(FetchContent)

FetchContent_Declare(
    yalantinglibs
    GIT_REPOSITORY https://github.com/HengXin666/HXLibs.git
    GIT_TAG main
    GIT_SHALLOW 1 # optional ( --depth=1 )
)

find_package(HXLibs CONFIG REQUIRED)
target_link_libraries(YouProject PRIVATE HXLibs::HXLibs)
```

#### 2.2.3 Cake add_subdirectory

1. 下载 HXLibs 的源代码, 将文件夹放到你的工程下。

2. 将 HXLibs 的根目录加入到你的工程中。

```cmake
add_subdirectory(libs/HXLibs) # 导入 libs 文件夹下的 HXLibs, *请根据你的需要进行修改
target_link_libraries(YouProject PRIVATE HXLibs)
```

#### 2.2.4 手动安装

> [!TIP]
> 待编写

### 2.3 配置选项

|工程选项|默认值|描述|
|:-:|:-:|:-|
|HXLIBS_ENABLE_SSL|OFF|开启后, 服务端将使用Https进行响应, 并且拒绝http请求.<br/>客户端可以选择使用 HttpClinet / HttpsClinet, 进行请求.|

## 三、第三方依赖清单
### 3.1 HX::net

|依赖库|说明|备注|
|:-:|:-:|:-:|
|liburing|io_uring的封装|https://github.com/axboe/liburing|
|OpenSSL 3.3.1+|用于https证书/握手/加解密|https://github.com/openssl/openssl|

其他的库仅使用了 STL / 操作系统头文件 内容. 无第三方依赖.

## 四、许可证

HXLibs 基于 Apache License (Version 2.0) 开发。