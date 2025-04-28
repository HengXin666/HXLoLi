# http服务端架构源码分析

## 一、api

```C++
cinatra::coro_http_server server(1 /*线程数*/, 9001 /*端口*/);

server.set_http_handler<cinatra::GET, cinatra::POST>("/delay2", [](
    coro_http_request &req,
    coro_http_response &resp
) -> async_simple::coro::Lazy<void> {
    resp.set_delay(true);
    std::this_thread::sleep_for(200ms);
    resp.set_status_and_content(status_type::ok, "delay reply in coro");
    co_await resp.get_conn()->reply();
});
```

还支持面向切面的编程:

```C++
//将信息从中间件传输到处理程序
struct get_data {
    bool before(coro_http_request& req, coro_http_response& res) {
        req.set_aspect_data("hello world");
        return true;
    }
}

server.set_http_handler<GET,POST>("/aspect/data", [](coro_http_request& req, coro_http_response& res) {
    std::string hello = req.get_aspect_data()[0];
    res.set_status_and_content(status_type::ok, std::move(hello));
}, get_data{} /*可注入多个切面*/);
```

---

## 二、具体实现

- 添加端点:

    1. 支持添加对不同的`method`到相同的端点(通过`...`展开)
    2. 小细节: 如果只有一个, 就采用移动!
    3. 路由是类独享的, 而不是全局单例!

```C++
template <http_method... method, typename Func, typename... Aspects>
  void set_http_handler(std::string key, Func handler, Aspects &&...asps) {
    static_assert(sizeof...(method) >= 1, "must set http_method");
    if constexpr (sizeof...(method) == 1) {
      (router_.set_http_handler<method>(std::move(key), std::move(handler),
                                        std::forward<Aspects>(asps)...),
       ...);
    }
    else {
      (router_.set_http_handler<method>(key, handler,
                                        std::forward<Aspects>(asps)...),
       ...);
    }
}
```

- 添加端点到路由:

```C++
template <http_method method, typename Func, typename... Aspects>
  void set_http_handler(std::string key, Func handler, Aspects&&... asps) {
    constexpr auto method_name = cinatra::method_name(method);
    std::string whole_str;
    whole_str.append(method_name).append(" ").append(key);

    // hold keys to make sure map_handles_ key is
    // std::string_view, avoid memcpy when route
    using return_type = typename util::function_traits<Func>::return_type;
    if constexpr (coro_io::is_lazy_v<return_type>) {
      std::function<async_simple::coro::Lazy<void>(coro_http_request & req,
                                                   coro_http_response & resp)>
          http_handler;
      if constexpr (sizeof...(Aspects) > 0) {
        http_handler = [this, handler = std::move(handler), ... asps = std::forward<Aspects>(asps)](
                           coro_http_request& req,
                           coro_http_response& resp) mutable
            -> async_simple::coro::Lazy<void> {
          bool ok = true;
          (do_before(asps, req, resp, ok), ...);
          if (ok) {
            co_await handler(req, resp);
          }
          ok = true;
          (do_after(asps, req, resp, ok), ...);
        };
      }
      else {
        http_handler = std::move(handler);
      }

      if (whole_str.find(":") != std::string::npos) {
        std::string method_str(method_name);
        coro_router_tree_->coro_insert(whole_str, std::move(http_handler),
                                       method_str);
      }
      else {
        if (whole_str.find("{") != std::string::npos ||
            whole_str.find(")") != std::string::npos) {
          std::string pattern = whole_str;

          if (pattern.find("{}") != std::string::npos) {
            replace_all(pattern, "{}", "([^/]+)");
          }

          coro_regex_handles_.emplace_back(std::regex(pattern),
                                           std::move(http_handler));
        }
        else {
          auto [it, ok] = coro_keys_.emplace(std::move(whole_str));
          if (!ok) {
            CINATRA_LOG_WARNING << key << " has already registered.";
            return;
          }
          coro_handles_.emplace(*it, std::move(http_handler));
        }
      }
    }
    else {
      std::function<void(coro_http_request & req, coro_http_response & resp)>
          http_handler;
      if constexpr (sizeof...(Aspects) > 0) {
        http_handler = [this, handler = std::move(handler),
                        ... asps = std::forward<Aspects>(asps)](
                           coro_http_request& req,
                           coro_http_response& resp) mutable {
          bool ok = true;
          (do_before(asps, req, resp, ok), ...);
          if (ok) {
            handler(req, resp);
          }
          ok = true;
          (do_after(asps, req, resp, ok), ...);
        };
      }
      else {
        http_handler = std::move(handler);
      }

      if (whole_str.find(':') != std::string::npos) {
        std::string method_str(method_name);
        router_tree_->insert(whole_str, std::move(http_handler), method_str);
      }
      else if (whole_str.find("{") != std::string::npos ||
               whole_str.find(")") != std::string::npos) {
        std::string pattern = whole_str;

        if (pattern.find("{}") != std::string::npos) {
          replace_all(pattern, "{}", "([^/]+)");
        }

        regex_handles_.emplace_back(std::regex(pattern),
                                    std::move(http_handler));
      }
      else {
        auto [it, ok] = keys_.emplace(std::move(whole_str));
        if (!ok) {
          CINATRA_LOG_WARNING << key << " has already registered.";
          return;
        }
        map_handles_.emplace(*it, std::move(http_handler));
      }
    }
}
```

支持面向切面的实现的核心思想: (将他们装在一起, 而不是分开)

```C++
调用体 = [...]() {
    bool ok = true;
    (do_before(asps, req, resp, ok), ...);
    if (ok) {
      handler(req, resp);
    }
    ok = true;
    (do_after(asps, req, resp, ok), ...);
};

路由.add(url, 调用体);
```

然后插入到路由树中(内部为迭代实现, 而非递归)