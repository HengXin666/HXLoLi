# Spring Boot 常用参数注解使用
### 1. `@PathVariable` 注解
通过 `@PathVariable` 可以将 URL 中占位符参数绑定到控制器处理方法的入参中。比如在 URL 中使用 `{}` 语法定义占位符。

```java
@RestController
public class ParamController {
    @RequestMapping("aa/{user}/mm/{num}")
    public String testPathVariable(
        @PathVariable("user") String userID, 
        @PathVariable("num") String num
    ) {
        System.out.println(userID);
        System.out.println(num);
        return "hello";
    }
}

// 发送请求: http://localhost:8088/aa/789/mm/333
```

**控制台输出**:
```text
789
333
```

### 2. `@RequestParam` 注解
`@RequestParam` 用于获取 URL 查询参数或表单参数。可以通过设置 `required` 和 `defaultValue` 属性来控制参数的必要性和默认值。

```java
@RestController
public class ParamController {
    @GetMapping("/greet")
    public String greet(@RequestParam(name = "name", required = false, defaultValue = "Guest") String name) {
        return "Hello, " + name;
    }
}

// 发送请求: http://localhost:8088/greet?name=John
```

**控制台输出**:
```text
Hello, John
```

### 3. `@RequestBody` 注解
`@RequestBody` 用于将请求体中的 JSON 数据绑定到 Java 对象。通常用于 POST 请求。

```java
@RestController
public class ParamController {
    @PostMapping("/user")
    public String createUser(@RequestBody User user) {
        System.out.println(user.getName());
        return "User created: " + user.getName();
    }
}

// 请求体: { "name": "Alice" }
```

### 4. `@CookieValue` 注解
`@CookieValue` 用于获取请求中的 Cookie 值。

```java
@RestController
public class ParamController {
    @GetMapping("/cookie")
    public String getCookieValue(@CookieValue(value = "SESSION", defaultValue = "default") String session) {
        return "Session ID: " + session;
    }
}

// 发送请求时需携带 SESSION Cookie
```

### 5. `@RequestHeader` 注解
`@RequestHeader` 用于获取请求头信息。

```java
@RestController
public class ParamController {
    @GetMapping("/header")
    public String getHeader(@RequestHeader("User-Agent") String userAgent) {
        return "User-Agent: " + userAgent;
    }
}
```

### 6. `@RequestAttribute` 注解
`@RequestAttribute` 用于获取请求中存储的属性，通常在拦截器中设置。

```java
@RestController
public class ParamController {
    @GetMapping("/attribute")
    public String getAttribute(@RequestAttribute("attrName") String attr) {
        return "Attribute: " + attr;
    }
}
```

### 7. `@MatrixVariable` 注解
`@MatrixVariable` 用于获取矩阵变量，矩阵变量是一种在路径中使用`分号/逗号`分隔的参数，适用于路径变量的进一步细分。

```java
@RestController
public class ParamController {
    @GetMapping("/users/{userId}")
    public String getUser(@PathVariable String userId, @MatrixVariable String age) {
        return "User ID: " + userId + ", Age: " + age;
    }
}

// 发送请求: http://localhost:8088/users/1;age=30
```
