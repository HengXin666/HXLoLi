# HX::net 简介

> `HX::net` 为 网络库, 使用 Http1.1 协议; 相关头文件存放于 `HXLibs/net`。

## 一、服务端
### 1.1 快速示例

- 一个简单的hello world:

```cpp
#include <HXLibs/net/server/HttpServer.hpp>

int main() {
    using namespace HX::net;

    HttpServer server{/*port:*/28205};
    server.addEndpoint<GET>("/", [] (Request& req, Response& res) {
        co_await res.setStatusAndContent(Status::CODE_200, "Hello World!")
                    .sendRes();
    });
    server.syncRun();
}
```

6行代码就可以实现一个简单http服务器了, 用户不需要关注多少细节, 直接写业务逻辑就行了。

### 1.2 认识 API 宏

为了方便用户使用, 我们提供了 API 宏:

```cpp
/**
 * @brief 定义标准的端点, 请求使用`req`变量, 响应使用`res`变量
 */
#define ENDPOINT (                           \
    [[maybe_unused]] HX::net::Request& req,  \
    [[maybe_unused]] HX::net::Response& res  \
) -> HX::coroutine::Task<>
```

这样可以简化端点的声明:

```cpp
#include <HXLibs/net/ApiMacro.hpp>

int main() {
    using namespace HX::net;

    HttpServer server{28205};
    server.addEndpoint<GET>("/", [] ENDPOINT {
        co_await res.setStatusAndContent(Status::CODE_200, "Hello World!")
                    .sendRes();
    });
    server.syncRun();
}
```

对于常见的 后端三层架构, 我们也提供了 **控制层** (Controller) 的声明宏, 以方便用户以统一的方式使用:

> 并且支持依赖注入

```cpp [server1-普通使用]
#include <HXLibs/net/ApiMacro.hpp> // 导入 HX::net 以及 API 宏

struct LoliDAO {
    uint64_t select(uint64_t id) {
        return id;
    }
};

HX_CONTROLLER(LoliController) {
    HX_ENDPOINT_MAIN(std::shared_ptr<LoliDAO> loliDAO) {
        using namespace HX::net;
        addEndpoint<GET>("/", [=] ENDPOINT {
            auto id = loliDAO->select(114514);
            co_await res.setStatusAndContent(Status::CODE_200, std::to_string(id))
                        .sendRes();
        });
        addEndpoint<POST>("/post", [=] ENDPOINT {
            auto id = loliDAO->select(2233);
            co_await res.setStatusAndContent(Status::CODE_200, std::to_string(id))
                        .sendRes();
        });
    }
};

#include <HXLibs/net/UnApiMacro.hpp> // undef 相关的宏

int main() {
    using namespace HX::net;
    HttpServer server{28205};

    // 依赖注入
    server.addController<LoliController>(std::make_shared<LoliDAO>());

    server.syncRun(1);
}
```

```cpp [server1-模板方法]
#include <HXLibs/net/ApiMacro.hpp>

struct LoliDAO {
    uint64_t select(uint64_t id) {
        return id;
    }
};

HX_CONTROLLER(LoliController) {

    // 如果您期望在依赖注入的时候, 注入模板类型 (比如期望CRTP实现编译期多态, 就需要使用如下 API 宏)
    HX_ENDPOINT_TEMPLATE_HEAD

    template <typename T>
    HX_ENDPOINT_TEMPLATE_MAIN(std::shared_ptr<T> loliDAO) {
        using namespace HX::net;
        addEndpoint<GET>("/", [=] ENDPOINT {
            auto id = loliDAO->select(114514);
            co_await res.setStatusAndContent(Status::CODE_200, std::to_string(id))
                        .sendRes();
        });
        addEndpoint<POST>("/post", [=] ENDPOINT {
            auto id = loliDAO->select(2233);
            co_await res.setStatusAndContent(Status::CODE_200, std::to_string(id))
                        .sendRes();
        });
    }
};

#include <HXLibs/net/UnApiMacro.hpp>

int main() {
    using namespace HX::net;
    HttpServer server{28205};

    // 依赖注入
    server.addController<LoliController>(std::make_shared<LoliDAO>());

    server.syncRun(1);
}
```

### 1.3 常用 API
#### 1.3.1 HttpServer

```cpp [HttpServer-添加端点]
/**
 * @brief 为服务器添加一个端点
 * @tparam Methods 请求类型, 如`GET`、`POST`; 如果不写, 则全部注册!
 * @tparam Func 端点函数类型
 * @tparam Interceptors 拦截器类型
 * @param key url, 如"/"、"home/{id}"
 * @param endpoint 端点函数
 * @param interceptors 拦截器
 * @return HttpServer& 可链式调用
 */
template <HttpMethod... Methods, typename Func, typename... Interceptors>
HttpServer& addEndpoint(std::string_view path, Func endpoint, Interceptors&&... interceptors);

/**
 * @brief 注册控制器到服务器, 可以进行依赖注入, 依次传参即可
 * @tparam T 控制器类型
 * @tparam Args 
 * @param args 被依赖注入的变量
 */
template <typename T, typename... Args>
inline HttpServer& addController(Args&&... args);
```

```cpp [HttpServer-启动服务器]
/**
 * @brief 同步启动 HttpServer
 * @tparam Timeout 字面常量, 表示超时时间 (单位: 秒(s))
 * @param threadNum 线程数
 * @param timeout 超时时间 (使用类型 utils::TimeNTTP)
 */
template <
    typename Timeout = decltype(utils::operator""_s<"30">()),
    typename Init = decltype([]{})
>
    requires(utils::HasTimeNTTP<Timeout>)
void syncRun(
    std::size_t threadNum = std::thread::hardware_concurrency(),
    Init&& init = Init{},
    Timeout timeout = {}
);

/**
 * @brief 异步启动 HttpServer
 * @warning 本方法不可重入, 并且线程不安全
 * @tparam Timeout 字面常量, 表示超时时间 (单位: 秒(s))
 * @param threadNum 线程数
 * @param timeout 超时时间 (使用类型 utils::TimeNTTP)
 */
template <
    typename Timeout = decltype(utils::operator""_s<"30">()),
    typename Init = decltype([]{})
>
    requires(utils::HasTimeNTTP<Timeout>)
void asyncRun(
    std::size_t threadNum = std::thread::hardware_concurrency(),
    Init&& init = Init{},
    Timeout = {}
);
```

```cpp [HttpServer-关闭服务器]
/**
 * @brief 同步关闭服务器
 * @warning 该方法不可重入
 */
void syncStop();

/**
 * @brief 同步关闭服务器
 * @warning 该方法不可重入
 */
void asyncStop();
```

#### 1.3.2 Request

```cpp [RequestServer-请求数据]
/**
 * @brief 获取请求类型
 * @return 请求类型 (如: "GET", "POST"...)
 */
std::string_view getReqType() const noexcept;

/**
 * @brief 获取请求PATH
 * @return 请求PATH (如: "/", "/home?loli=watasi"...)
 */
std::string_view getReqPath() const noexcept;

/**
 * @brief 获取请求协议版本
 * @return 请求协议版本 (如: "HTTP/1.1", "HTTP/2.0"...)
 */
std::string_view getProtocolVersion() const noexcept;

/**
 * @brief 获取请求头键值对的引用
 * @return HeaderHashMap 
 */
const auto& getHeaders() const noexcept;

/**
 * @brief 获取 Range 请求视图 (一般配合 `res.useRangeTransferFile()` 使用) 
 * @return RangeRequestView 
 */
RangeRequestView getRangeRequestView() const;

/**
 * @brief 朴素的解析 Body
 * @tparam Timeout 超时时间
 * @return Body String
 */
template <typename Timeout = decltype(utils::operator""_s<"5">())>
    requires(utils::HasTimeNTTP<Timeout>)
coroutine::Task<std::string> parseBody();

/**
 * @brief 解析 Body 并且保存到 path
 * @param path 保存路径
 */
template <typename Timeout = decltype(utils::operator""_s<"5">())>
    requires(utils::HasTimeNTTP<Timeout>)
coroutine::Task<> saveToFile(std::string_view path);
```

```cpp [RequestServer-参数解析]
/**
 * @brief 解析查询参数 (解析如: `?name=loli&awa=ok&hitori`)
 * @return 返回解析到的字符串键值对哈希表
 * @warning 如果解析到不是键值对的, 即通过`&`分割后没有`=`的, 默认其全部为Key, 但Val = ""
 */
std::unordered_map<std::string, std::string> getParseQueryParameters() const;

/**
 * @brief 获取请求的纯PATH部分
 * @return 请求PATH (如: "/", "/home?loli=watasi"的"/home"部分)
 */
std::string getPureReqPath() const noexcept;

/**
 * @brief 获取第`index`个路径参数的内容
 * @param index 路径参数索引, 如`/home/{name}/id`, `index = 0` => {name}
 * @throw std::runtime_error 如果路径参数未初始化则抛出 (端点函数必须为`{val}`格式)
 * @throw std::out_of_range 如果 `index` 超出可用路径参数范围时抛出
 * @note 调用前需要确保路径参数已正确初始化
 * @return std::string_view 
 */
PathParam getPathParam(std::size_t index) const;

/**
 * @brief 获取通配符路径参数的内容
 * @throw std::runtime_error 如果路径参数未初始化则抛出 (端点函数必须为`{val}`格式)
 * @return std::string_view 
 */
std::string_view getUniversalWildcardPath() const;
```

#### 1.3.3 Response

```cpp [ResponseServer-设置响应]
/**
 * @brief 设置状态行 (协议使用HTTP/1.1)
 * @param statusCode 状态码
 * @param describe 状态码描述: 如果为`""`则会使用该状态码对应默认的描述
 * @warning 不需要手动写`/r`或`/n`以及尾部的`/r/n`
 */
HttpResponse& setResLine(Status statusCode, std::string_view describe = "");

/**
 * @brief 设置响应头部: 设置响应类型, 如果响应体是文本, 你需要指定字符编码(不指定则留空`""`)
 * @param type 响应类型, 如`text/html`
 * @param encoded 字符编码, 如`UTF-8` ~~(如果是图片等就可以不用指定)~~
 * @return [this&] 可以链式调用
 * @warning 不需要手动写`/r`或`/n`以及尾部的`/r/n`
 */
HttpResponse& setContentType(HttpContentType type);

/**
 * @brief 向响应头部添加一个键值对
 * @param key 键
 * @param val 值
 * @return Response&
 * @warning `key`在`map`中是区分大小写的, 故不要使用`大小写不同`的相同的`键`
 */
template <typename Str>
HttpResponse& addHeader(const std::string& key, Str&& val);

/**
 * @brief 设置响应体
 * @warning 不需要手动写`/r`或`/n`以及尾部的`/r/n`
 * @tparam S 字符串类型
 * @param data 信息
 * @return Request& 
 */
template <typename S>
    requires (requires (S&& data, std::string s) {
        s += std::forward<S>(data);
    })
HttpResponse& setBody(S&& data) noexcept;

/**
 * @brief 设置响应码和正文(html)
 * @param status 
 * @param content
 * @return Response& 可链式调用
 */
HttpResponse& setStatusAndContent(Status status, std::string_view content);
```

```cpp [ResponseServer-发送响应]
/**
 * @brief 发送已经设置的响应
 * @return coroutine::Task<> 
 */
coroutine::Task<> sendRes();

/**
 * @brief 使用分块编码传输文件
 * @note 内部会自行设置 响应头 (Content-Type、Transfer-Encoding)
 * @param filePath 文件路径
 */
coroutine::Task<> useChunkedEncodingTransferFile(std::string_view filePath);

/**
 * @brief 使用断点续传传输文件
 * @note 内部会智能判断客户端是否需要使用断点续传, 最坏也只是降级为普通传输 (都是分块读和发的)
 * @note 内部会自行设置 响应头 (Content-Type、Content-Length)
 * @param rrv 断点续传参数包, 通过 `req.getRangeRequestView()` 获取
 * @param filePath 文件路径
 */
coroutine::Task<> useRangeTransferFile(RangeRequestView rrv, std::string_view filePath);
```

#### 1.3.4 WebSocket

```cpp [WebSocketServer-API]
/**
 * @brief WebSocket的工厂方法
 */
class WebSocketFactory {
public:
    /**
     * @brief 服务端连接, 并且创建 ws 对象
     * @param req 
     * @param res 
     * @return coroutine::Task<WebSocket> 
     */
    static coroutine::Task<WebSocketServer> accept(Request& req, Response& res);

    /**
     * @brief 生成一个可以复用的发送包, 以减少拷贝开销
     * @warning 仅服务端可用
     * @param opCode 
     * @param msg 
     * @return WebSocketPacketView 
     */
    static WebSocketServerSendView makePacketView(OpCode opCode, std::string_view msg);
};
```

```cpp [WebSocketServer-消息池示例]
#include <HXLibs/net/ApiMacro.hpp>
#include <HXLibs/net/protocol/websocket/WebSocket.hpp>

// WebSocket 消息池
// 正确使用应该是 一个 ws 接口专门用来发送消息
// 而发送数据则应该使用另一个接口, 也就是不能使用这个ws
struct WSPool {
    coroutine::Task<> sendAll(std::string_view msg) {
        if (wsPool.empty())
            co_return;
        // 一次生成数据
        auto pk = WebSocketFactory::makePacketView(OpCode::Text, msg);
        for (auto& ws : wsPool) {
            // 多次重发这个数据
            co_await ws.sendPacketView(pk);
        }
    }
    std::list<WebSocket<WebSocketModel::Server>> wsPool;
};

int main() {
    WSPool pool;
    HttpServer serv{28205};
    serv.addEndpoint<GET>("/ws", [] ENDPOINT {
        auto ws = co_await WebSocketFactory::accept(req, res);
        struct JsonDataVo {
            std::string msg;
            int code;
        };
        JsonDataVo const vo{"Hello 客户端, 我只能通信3次!", 200};
        co_await ws.sendJson(vo);
        for (int i = 0; i < 3; ++i) {
            auto res = co_await ws.recvText();
            log::hxLog.info(res);
            co_await ws.sendText("Hello! " + res);
        }
        co_await ws.close();
        log::hxLog.info("断开ws");
        co_return;
    })
    .addEndpoint<GET>("/ws_add_msg/{msg}", [&] ENDPOINT {
        // 群发内容
        std::string_view msg = req.getPathParam(0);
        co_await pool.sendAll({msg.data(), msg.size()});
        co_await res.setResLine(HX::net::Status::CODE_200)
                    .sendRes();
        co_return;
    })
    .addEndpoint<GET>("/ws_send_poll", [&] ENDPOINT {
        auto ws = co_await WebSocketFactory::accept(req, res);
        auto it = pool.wsPool.emplace(pool.wsPool.end(), ws);
        try {
            while (true) {
                // 仅维持心跳, 客户端不应该发送除了 ping 以外的任何内容
                co_await ws.recvText();
            }
        } catch (...) {
            // ws连接已断开
        }
        // 注意线程安全啊! 这里是单线程, 仅示例, 故没有封装和上锁
        pool.wsPool.erase(it);
    });
    serv.syncRun(1, 1500_ms); // 启动服务器
}
```

## 二、客户端

## 三、WebSocket