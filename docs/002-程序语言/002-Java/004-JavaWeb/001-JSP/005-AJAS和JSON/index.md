<style>
/* Markdown风格的样式 */

/* h1的样式 */
h1 {
    color: yellow;
    margin-top: 1.5em; /* 顶部间距 */
    margin-bottom: 0.5em; /* 底部间距 */
}

/* h2的样式 */
h2 {
    color: rgb(100,233,233);
    margin-top: 1.25em; /* 顶部间距 */
    margin-bottom: 0.5em; /* 底部间距 */
}

/* h3的样式 */
h3 {
    color: rgb(250, 100, 200);
    margin-top: 1.25em; /* 顶部间距 */
    margin-bottom: 0.5em; /* 底部间距 */
}

/* h4的样式 */
h4 {
    color: rgb(75,250,75);
    margin-top: 1.25em; /* 顶部间距 */
    margin-bottom: 0.5em; /* 底部间距 */
}

/* 段落样式 */
p {
    margin-top: 1em; /* 顶部间距 */
    margin-bottom: 1em; /* 底部间距 */
    text-indent: 1.5em; /* 首行缩进 */
}
</style>
# AJAX
## 什么是ajax
AJAX 全称为 **Asynchronous JavaScript And Xml**，表示异步的Java脚本和 Xml 文件，是一种**异步刷新技术**。

## 为什么要使用ajax
`Servlet`进行网页的变更往往是通过**请求转发**或者是**重定向**来完成，这样的操作更新的是整个网页，如果我们只需要更新网页的局部内容，就需要使用到AJAX来处理了。因为只是更新局部内容，因此，`Servlet`传输的数据量就减少了，这不仅有效的利用了带宽，提高效率的同时还增加了用户的体验度，操作起来更为方便。

## AJAX的核心
- IE浏览器: `ActiveXObject`
- 其他浏览器: `XMLHttpRequest`

*AJAX的核心是一个对象，既然是对象，那么就应该存在属性和方法。*

**常用方法**:

| 方法              | 说明                                      |
|------------------|-------------------------------------------|
| open(String method, String url, boolean async) | 创建一个新的HTTP请求                      |
| send(String data) | 发送请求到服务器端                        |
| setRequestHeader(String header, String value) | 设置请求的某个HTTP头信息                  |

**常用属性**:

- `onreadystatechange`: 监听就绪状态改变的事件，必须给定一个函数
- `readyState`: `XMLHttpRequest`的状态信息
- `status`: HTTP的状态码
- `responseText`: 以文本形式获得响应的内容
- `responseXML`: 将XML格式的响应内容解析成DOM对象

## 应用场景(示例)
### 用户名检测
使用 ajax 完成注册时用户名是否可用

注册界面
```html (jsp)
<input type="text" id="username"><span style="color: red"></span>

<script type="text/javascript" src="js/jquery-3.7.1.js"></script>
<script type="text/javascript">
    $("#username").blur(() => { // 添加一个失去焦点事件
        let xhr; // 创建ajax引擎对象, 所有操作都是由ajax引擎完成
        if (window.XMLHttpRequest) { // GOOGLE, FIREFOX, IE7以上
            xhr = new XMLHttpRequest();
        } else if (window.ActiveXObject) { // IE7以下
            xhr = new ActiveXObject("Microsoft.XMLHTTP");
        }
        xhr.onreadystatechange = () => { // 为引擎对象绑定就绪状态监听事件
            if(xhr.readyState == 4 && xhr.status == 200) {
                if(xhr.responseText === "-1") {
                    $("#username").siblings("span").text("该账号已被注册");
                    $("#username").siblings("span").css("color", "red");
                } else {
                    $("#username").siblings("span").text("很好的名字");
                    $("#username").siblings("span").css("color", "yellow");
                }
            }
        };
        // 第一个参数：请求方式GET/POST，第二个参数：后台服务器地址，第三个参数：是否异步
        xhr.open("GET", "ajaxText1?username=" + $("#username").val(), true);
        xhr.send(); // 发送请求
    });
</script>
```

后端代码

```java
package com.HX.jsp.servlet;

import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import java.io.IOException;
import java.io.PrintWriter;

@WebServlet(urlPatterns = "/ajaxText1")
public class AjaxTextOne extends HttpServlet {
    @Override
    protected void doGet(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
        String username = req.getParameter("username");
        System.out.println("接收: " + username);
        int res = 0;
        if ("hx".equals(username)) { // 模拟通过数据库查询...
            res = -1;
        }
        PrintWriter writer = resp.getWriter();
        writer.print(res);
        writer.flush();
        writer.close();
    }
}
```

### 登录
使用 ajax 完成登录


```html
<div>
    <input type="text" id="loginUsername">
</div>
<div>
    <input type="password" id="password">
</div>
<div>
    <input type="button" value="登录" id="loginBtn">
</div>
<script type="text/javascript">
    $("#loginBtn").click(function () {
        let xhr; // 创建ajax引擎对象，所有操作都是由ajax引擎完成
        if(window.XMLHttpRequest) { // GOOGLE，FIREFOX，IE7以上
            xhr = new XMLHttpRequest();
        } else if(window.ActiveXObject) { // IE7以下
            xhr = new ActiveXObject("Microsoft.XMLHTTP");
        }
        xhr.onreadystatechange = function () {//为引擎对象绑定就绪状态监听事件
            if(xhr.readyState == 4 && xhr.status == 200) {
                if(xhr.responseText === "1") {
                    alert("登录成功")
                } else {
                    alert("登录失败")
                }
            }
        }
        // 第一个参数：请求方式GET/POST，第二个参数：后台服务器地址，第三个参数：是否异步
        xhr.open("POST","ajaxText2", true);
        // POST方式发送请求时，携带数据一定要添加请求头，设置传输数据的类型
        // 为application/x-www-form-urlencoded其作用将键值对的参数用&连接起来，
        // 如果有空格，将空格转换为+加号；有特殊符号，将特殊符号转换为ASCII HEX值
        xhr.setRequestHeader('content-type', 'application/x-www-form-urlencoded;charset=UTF-8');
        xhr.send("loginUsername=" + $("#loginUsername").val()
            + "&password=" + $("#password").val()); // 发送请求
    });
</script>
```

后端
```java
package com.HX.jsp.servlet;

import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import javax.servlet.http.Part;
import java.io.IOException;
import java.io.PrintWriter;

@WebServlet(urlPatterns = "/ajaxText2")
public class AjaxTextTow extends HttpServlet {
    @Override
    protected void doPost(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
        String loginUsername = req.getParameter("loginUsername");
        String password = req.getParameter("password");
        int res = 0;
        System.out.println("登录: " + loginUsername + " " + password);
        if ("hx".equals(loginUsername) && "123456".equals(password))
            res = 1;
        PrintWriter writer = resp.getWriter();
        writer.print(res);
        writer.flush();
        writer.close();
    }
}
```

## 自行封装AJAX
> 自己搞

# jQuery ajax
## 为什么要使用 jQuery ajax
原生 ajax 使用步骤繁琐，接收的数据格式需要处理，使用过程中涉及到的函数调用及状态验证较多，还需要处理浏览器的兼容性问题。而 jQuery ajax 对原生 ajax 进行了封装，使用起来非常方便，深受开发者的喜爱。

## 如何使用 jQuery ajax

```js (jQuery)
$.ajax({
    url: '',         // 请求提交的URL地址
    type: '',        // 请求类型
    data: {},        // 请求携带的数据
    contentType: '', // 请求的数据类型
    dataType: '',    // 服务器端传回的数据类型，默认是application/text
    // 成功时执行的回调函数，resp用于接收服务器端传递回来的数据
    success: function (resp) {},
    // 请求过程中出现错误后执行的回调函数，xhr用于接收失败后相关状态信息
    error: function(xhr, textStatus, errorThrown) {}
});
```

## 应用场景(示例)
### 用户名检测
此处直接改于上面的代码:
```html (jsp)
<script type="text/javascript">
    $(()=>{
        $("#username").blur(() => {$.ajax({
           url: "ajaxText1",
           type: "get",
           data: {
               username: $("#username").val()
           },
           contentType: 'application/x-www-form-urlencoded;charset=UTF-8',
           success: (resp) => { // resp用于接收服务器端传递回来的数据
               if(resp === "-1") {
                   $("#username").siblings("span").text("该账号已被注册");
                   $("#username").siblings("span").css("color", "red");
               } else {
                   $("#username").siblings("span").text("很好的名字");
                   $("#username").siblings("span").css("color", "yellow");
               }
           }
       })}); // <-- 没有在上面换行的下场
    });
</script>
```

### 登录

```html (jsp)
<script type="text/javascript">
    $(() => {
        $("#loginBtn").click(() => {
            $.ajax({
                url: "ajaxText2",
                type: "post",
                data: {
                    loginUsername: $("#loginUsername").val(),
                    password: $("#password").val()
                },
                contentType: "application/x-www-form-urlencoded;charset=UTF-8",
                success: (resp) => {
                    if(resp === "1") {
                        alert("登录成功")
                    } else {
                        alert("登录失败")
                    }
                }
            })
        })
    })
</script>
```

## 表格刷新
表格刷新主要利用的是 jQuery 的`load()`方法进行表格刷新，这也是一种`ajax`的封装。`load()`方法可以**对页面的局部内容进行更新**，`load()`方法使用有两种方式:

```js (jQuery)
// 这种方式是GET请求
$("选择器").load("请求地址");

// 这种方式是POST请求
$("选择器").load("请求地址", data);
```

示例:

```html (jsp)
<!-- table.jsp -->
<input type="text" id="region">
<input type="button" value="查询" id="search">

<table>
    <thead>
    <tr>
        <th>代理商ID</th>
        <th>代理商编号</th>
        <th>代理商名称</th>
        <th>代理商区域</th>
    </tr>
    </thead>
    <tbody id="dataBox"></tbody>
</table>
<script type="text/javascript">
    $(function () {
        $("#search").click(function () {
            let data = { region: $("#region").val() };
            $("#dataBox").load("search", data);
        });
    })
</script>
```

```html (jsp)
<!-- data.jsp -->
<%@ page contentType="text/html;charset=UTF-8" language="java" %>
<%@ taglib prefix="c" uri="http://java.sun.com/jsp/jstl/core" %>
<c:forEach items="${agents}" var="agent">
    <tr>
        <td>${agent.aid}</td>
        <td>${agent.ano}</td>
        <td>${agent.aname}</td>
        <td>${agent.aregion}</td>
    </tr>
</c:forEach>
```

后端

```java
@WebServlet(urlPatterns = "/search")
public class searchServet extends HttpServlet {
    @Override
    protected void doPost(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
        System.out.println("收到post");
        String region = req.getParameter("region");
        List<Agent> agents = Agent.queryAgents(region);
        System.out.println(Arrays.toString(agents.toArray()));
        req.getSession().setAttribute("agents", agents);
        resp.sendRedirect("data.jsp");
    }
}

public class Agent {
    private String aid;
    private String ano;
    private String aname;
    private String aregion;

    public Agent(int aid, int ano, String aname, String aregion) {
        this.aid = String.valueOf(aid);
        this.ano = String.valueOf(ano);
        this.aname = aname;
        this.aregion = aregion;
    }

    public static List<Agent> queryAgents(String region) {
        List<Agent> res = new ArrayList<>();
        int len = (new Random()).nextInt(20) + 1;
        for (int i = 1; i <= len; ++i) {
            res.add(new Agent(i + 100, i, "销售商" + i, region));
        }
        return res;
    }

    // get / set / toString 略
}
```

# JSON
## 什么是 JSON
JSON 全称为 **JavaScript Object Notation**，表示 **Javascript** 对象符号，是一种网络数据交换的格式，通常在服务器端和客户端之间使用。

## 如何定义 JSON
语法:

```js (json)
// JSON格式的对象
let json = {
    "属性名1": "属性值1",
    "属性名2": "属性值2",
    "属性名3": "属性值3",
    ...
};

// JSON格式的数组
let jsonArray = [值1,值2,值3, ...];

// JSON格式的对象数组
let jsonObjectArray = [
    {"属性名1": "数值1","属性名2": "数值2", ...},
    {"属性名1": "数值1","属性名2": "数值2", ...}
]
```

## Servlet 返回 JSON 格式数据
0. 手动写一个`toJson()`方法
    - 例如: `return "{\"name\":" + this.name + "\"}";` 显然太麻烦了.
1. 引入`fastJson`jar包

示例: (将上面的表格刷新使用`$.ajax`)

```html (jsp)
<script type="text/javascript">
    $(function () {
        $("#search").click(() => { // 修改了
            // let data = { region: $("#region").val() };
            // $("#dataBox").load("search", data);
            $.ajax({
                url: "search",
                type: "post",
                data: {
                    region: $("#region").val()
                },
                success: (resp) => {
                    let date = $("#dataBox");
                    date.empty();
                    for (let i = 0; i < resp.length; ++i) {
                        let tr = $("<tr></tr>");
                        tr.append($("<td>"+ resp[i].aid + "</td>"));
                        tr.append($("<td>"+ resp[i].ano + "</td>"));
                        tr.append($("<td>"+ resp[i].aname + "</td>"));
                        tr.append($("<td>"+ resp[i].aregion + "</td>"));
                        date.append(tr);
                    }
                }
            });
        });
    })
</script>
```

后端

```java
@WebServlet(urlPatterns = "/search")
public class searchServet extends HttpServlet {
    @Override
    protected void doPost(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
        String region = req.getParameter("region");
//        resp.setCharacterEncoding("UTF-8"); (已经配置了过滤器自动设置为utf-8了)
        resp.setContentType("application/json"); // 这一行是关键!!!
        List<Agent> agents = Agent.queryAgents(region);
        PrintWriter writer = resp.getWriter();
        writer.print(JSONObject.toJSON(agents));
        writer.flush();
        writer.close();
//        req.getSession().setAttribute("agents", agents);
//        resp.sendRedirect("data.jsp");
    }
}
```
