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

# EL表达式
## 什么是 EL 表达式
EL 全称为 **Expression Language（表达式语言）**

## 为什么要使用 EL 表达式
在 jsp 页面中编写小脚本，会存在以下不足:
- 代码结构混乱
- 脚本与HTML混合，容易出错
- 代码不易于维护
- 获取JavaBean属性必须要实例化及强制类型转化

为了解决这些不足，JSP提供了 EL 表达式来简化编码，可以使用 EL 表达式来替换 jsp 页面中的小脚本，使得页面和业务逻辑处理相分离，同时还能实现数据类型的自动转型

## 如何使用 EL 表达式
### EL 获取变量的值
语法:

```EL
${变量名}
```

案例:

1. 编写一个HttpServlet
```java
@Override
protected void doPost(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
    HttpSession session = req.getSession();
    session.setAttribute("user", "张三");
    req.getRequestDispatcher("elText.jsp").forward(req, resp); // 使用重定向
}
```

2. 编写展示信息的
```html (jsp)
<%@ page contentType="text/html;charset=UTF-8" language="java" %>
<html>
<head>
    <title>EL表达式</title>
</head>
<body>
当前用户名: ${user}
</body>
</html>
```

3. 编写一个访问上面HttpServlet的

```html
<div>
    <form action="el" method="post">
        <input type="submit" value="el测试">
    </form>
</div>
```

4. 启动测试, 发现可以正常显示
5. 将代码改为`转发`:

```java
req.setAttribute("user", "李四");
req.getRequestDispatcher("elText.jsp").forward(req, resp); // 使用转发
```

6. 依旧正常显示

**结论:** 不论变量是存储在`request`中，还是存储在`session`中，都可以使用 EL 表达式进行取值。

**質問:** 如果`request`和`session`中都存有相同的属性值，那么使用 EL 取值的时候，是从哪个对象中取值呢？

答案是: 使用`request`的
```java
HttpSession session = req.getSession();
session.setAttribute("user", "张三");
req.setAttribute("user", "李四");
req.getRequestDispatcher("elText.jsp").forward(req, resp); // 使用转发
```

那如果我们需要`session`的怎么办? 见下:

### EL 隐式对象
| **对象名称** | **说明** |
|------------|---------|
| pageScope | 返回页面范围的变量名，这些名称已映射至相应的值 |
| requestScope | 返回请求范围的变量名，这些名称已映射至相应的值 |
| sessionScope | 返回会话范围的变量名，这些名称已映射至相应的值 |
| applicationScope | 返回应用范围内的变量，并将变量名映射至相应的值 |
| param | 返回客户端的请求参数的字符串值 |
| paramValues | 返回映射至客户端的请求参数的一组值 |
| pageContext | 提供对用户请求和页面信息的访问 |

实际上我们常用的只有前4个, 使用也肥肠简单:

```html (el)
<!-- 在session范围内取值 -->
${sessionScope.user}
```
如果 EL **未指定隐式对象**，则
- 取值默认从`pageScope`取值，
- 如果未找到，则从`requestScope`取值，
- 如果还是未找到，则从`sessionScope`取值，
- 如果依然未找到，则从`applicationScope`取值，
- 如果最终都未找到，那么返回`null`值

### EL 获取对象的属性值
语法: 
- ***注意: 使用的属性名必需实现`get`方法, `public`也不行***
```html (el)
<!-- Java中的访问方式 -->
${ 对象名.属性名 }

<!-- JS中的访问方式 -->
${ 对象名["属性名"] }
```

### EL 获取 List 集合中的值
语法:

```html (el)
${ 集合名称[下标] }
```

### EL 获取 Map 集合中的值
语法:

```html (el)
<!-- Java中的访问方式 -->
${ 集合名称.键名 }

<!-- JS中的访问方式 -->
${ 集合名称["键名"] }
```

### EL 表达式中的操作符

| 关系操作符 | 说明       | 示例              | 结果   |
|--------------|------------|-------------------|--------|
| == (或eq)   | 等于        | ${23==5} ${23 eq 5} | false  |
| != (或ne)   | 不等于      | ${23!=5} ${23 ne 5} | true   |
| < (或lt)    | 小于        | ${23<5} ${23 lt 5}  | false  |
| > (或gt)    | 大于        | ${23>5} ${23 gt 5}  | true   |
| <= (或le)   | 小于等于    | ${23<=5} ${23 le 5} | false  |
| >= (或ge)   | 大于等于    | ${23>=5} ${23 ge 5} | true   |

| 逻辑操作符 | 说明       | 示例                          | 结果  |
|------------|------------|-------------------------------|-------|
| && (或and) | 逻辑与      | 如果A为true，B为false，则A&&B(或A and B) | false |
| \|\| (或or)  | 逻辑或      | 如果A为true，B为false，则A\|\|B(或A or B)  | true  |
| ! (或not)  | 逻辑非      | 如果A为true，则!A(或not A)             | false |

| Empty操作符 | 说明   | 示例               | 结果   |
|-------------|--------|--------------------|--------|
| empty       | 不存在 | ${empty a}，如果a存在 | false  |
| not empty   | 存在   | ${not empty a}，如果a存在 | true   |

示例:

```java
@Override
protected void doPost(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
    HttpSession session = req.getSession();
    session.setAttribute("user", new User("老八", 23, "男"));
    req.setAttribute("user", new User[] {
            new User("老六", 27, "男"),
            new User("老哥", 21, "男")
    });
    HashMap<String, String> map = new HashMap<>();
    map.put("loli", "imouto");
    req.setAttribute("himitsu", map);
    req.getRequestDispatcher("elText.jsp").forward(req, resp); // 使用转发
}
```

```html (jsp)
<%@ page contentType="text/html;charset=UTF-8" language="java" %>
<html>
<head>
    <title>EL表达式</title>
</head>
<body>
当前用户名: ${sessionScope.user.name}_${sessionScope.user["sex"]}, ${requestScope.user[0].name}
<br>
>> ${requestScope.himitsu["loli"]}
<br>
>> ${1 == 1 && 2 != 3 || 0 == 1e10 && empty a}
</body>
</html>
```

显示:

```C OUT
当前用户名: 老八_男, 老六
>> imouto
>> true
```

# JSTL标签
## 什么是 JSTL
JSTL 全称为 **JavaServerPages Standard Tag Library**，意味 JSP标准标签库

## 为什么要使用JSTL
EL 能够简化 jsp 页面编码，但是，却不能进行逻辑判断，也不能进行循环处理，为了弥补 EL 这方面的不足，jsp 提供了 JSTL 标签，JSTL 标签通常都会与 EL 配合使用，解决页面的逻辑问题。

## JSTL 标签库分类
| 标签库名称         | 资源标示符 (URI)                   | 前缀   |
|-------------------|-----------------------------------|--------|
| 核心标签库        | http://java.sun.com/jsp/jstl/core | c      |
| 国际化/格式化标签库 | http://java.sun.com/jsp/jstl/fmt  | fmt    |
| XML标签库         | http://java.sun.com/jsp/jstl/xml  | x      |
| 数据库标签库       | http://java.sun.com/jsp/jstl/sql  | sql    |
| 函数标签库        | http://java.sun.com/jsp/jstl/functions | fn |

经常使用的标签就是 **核心标签库** 和 **格式化标签库**

## JSTL 标签库的使用步骤
1. 引入JSTL标签库支持的jar包: `jstl.jar`和`standard.jar`
2. jsp 页面引入标签库，如

```jsp
<%@ taglib uri="http://java.sun.com/jsp/jstl/core" prefix="c"%>
```

## JSTL 核心标签库
### 通用标签
| 标签类别   | 标签名   | 说明                                      |
|-------------|----------|-------------------------------------------|
| 通用标签   | set      | 设置指定范围内的变量名和变量值             |
| 通用标签   | out      | 输出变量的值                              |
| 通用标签   | remove   | 删除指定范围内的变量                      |

#### < c:set > 标签
语法:

```html (jstl)
<!-- 将value值存储到范围为scope的变量variable中 -->
<c:set var="变量名" value="变量值" scope="变量的作用范围" />

<!-- 将value值设置到对象的属性中 -->
<c:set target="目标对象" property="对象属性" value="对象属性值" />
```

示例: *注意, 如果`""`内需要使用字符串, 请使用`\`进行转义, 如下*

```html (jstl)
<%
    request.setAttribute("hx", new User("张三", 12, "飞机"));
%>
<c:set var="user" value="<%=new User(\"老六\", 22, \"炸弹\")%>" scope="page"/>
<c:set target="${pageScope.user}" property="name" value="admn"/>
<div>
    页面中的${pageScope.user.name}
    <br>
    请求中的${requestScope.hx.name}
</div>
```

#### < c:remove > 标签
语法:

```html (jstl)
<c:remove var="变量名" scope="变量的作用范围" />
```

案例:

```html (jstl)
<c:remove var="user" scope="page"/>
删除后: 页面中的 ${pageScope.user.name}
```

### 条件标签
#### < c:if > 标签
语法:
```html (jstl)
<c:if test="条件表达式" var="存储表达式的结果的变量" scope="变量的作用范围">
<!-- 如果为真, 则显示这里的内容 -->
</c:if>
```

示例:

```html (jstl)
<div>
    <c:if test="${1 == 1}" var="awa" scope="page">
        <p>1 == 1</p>
    </c:if>
    结果是 ${pageScope.awa}
</div>
```

#### < c:choose > 标签
语法:

```html (jstl)
<c:choose>
    <c:when test="条件表达式"></c:when> <!--else if (条件)-->
    <c:when test="条件表达式"></c:when> <!--else if (条件)-->
    <c:otherwise></c:otherwise> <!--else-->
</c:choose>
```

示例:

```html (jstl)
<div>
    <c:set var="awa" value="1"/>
    <c:choose>
        <c:when test="${awa == 1}">
            确实: awa == 1
        </c:when>
        <c:when test="${awa > 1}">
            确实: awa > 1
        </c:when>
        <c:otherwise>
            确实: awa < 1
        </c:otherwise>
    </c:choose>
</div>
```

#### < c:forEach > 标签
语法:

```html (jstl)
<c:forEach items="遍历的集合" var="每次遍历的对象" begin="遍历开始的位置" end="遍历结束的位置" step="遍历的步长">
</c:forEach>

<!--begin / end / step 是可选的, 不写则默认 [0, n) ++i 的遍历-->
```

示例:

```html (jstl)
<div>
    <%
        List<String> arr = new ArrayList<>();
        for (int i = 1; i < 20; ++i) {
            arr.add("老" + i);
        }
        request.setAttribute("ggg", arr);
    %>
    <c:forEach items="${requestScope.ggg}" var="it">
        ${it} <br>
    </c:forEach>
</div>
```

```html (jstl)
<%
    Map<String,Integer> scores = new HashMap<String,Integer>();
    for(int i=0; i<10; i++){
        scores.put("test"+i ,i*10);
    }
    session.setAttribute("scores", scores);
%>
<c:forEach items="${scores}" var="map">
    <div>${map.key}=>${map.value}</div>
</c:forEach>
```

## JSTL 格式化标签库
| 标签类别   | 标签名       | 说明                  |
|-------------|--------------|-----------------------|
| 格式化标签 | formatDate   | 对日期进行格式化       |
| 格式化标签 | formatNumber | 对数字进行格式化       |

### < fmt:formatDate > 标签
语法:

```html (jstl)
<fmt:formatDate value="日期对象" pattern="日期格式" />
```

案例:

```html (jstl)
<%
    request.setAttribute("time", new Date());
%>
<fmt:formatDate value="${requestScope.time}" pattern="yyyy-MM-dd HH:mm:ss"/>
```

### < fmt:formatNumber > 标签
语法:

```html (jstl)
<!-- 货币格式的数字 -->
<fmt:formatNumber value="数字" type="currency" />
  
<!-- 数字格式化 -->
<fmt:formatNumber value="数字" type="number" maxIntegerDigits="整数部分位数" />
<fmt:formatNumber value="数字" type="number" maxFractionDigits="小数部分位数"/>
<fmt:formatNumber value="数字" type="number" pattern="数字格式" />

<!-- 数字百分比 -->
<fmt:formatNumber value="数字" type="percent" maxIntegerDigits="整数部分位数"/>
<fmt:formatNumber value="数字" type="percent" maxFractionDigits="小数部分位数"/>
```

示例:

```html (jstl)
<div>
    货币格式：<fmt:formatNumber type="currency" value="${number}" />
</div>
<div>
    数字格式化：<fmt:formatNumber type="number" maxIntegerDigits="3" value="${number}" />
    数字格式化：<fmt:formatNumber type="number" maxFractionDigits="3" value="${number}" />
    数字格式化：<fmt:formatNumber type="number" value="${number}" pattern="####.##"/>
</div>
<div>
    数字百分比：<fmt:formatNumber type="percent" maxIntegerDigits="3" value="${number}" />
    数字百分比：<fmt:formatNumber type="percent" maxFractionDigits="3" value="${number}" />
</div>
```
