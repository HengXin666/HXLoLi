# jQuery 操作元素
## 操作元素内容
```js
$(选择器|DOM元素).text(); // 获取标签内的文本内容
$(选择器|DOM元素).text(元素文本内容); // 设置标签内的文本内容

$(选择器|DOM元素).html(); // 获取标签内的html内容
$(选择器|DOM元素).html(元素内html内容); // 设置标签内的html内容
```

示例
```html
<body>
    <div id="error"></div>
    <div id="info"></div>
</body>
<script src="./js/jquery-3.7.1.js"></script>
<script>
$(() => {
    let error = $("#error");
    // 以下三种是等价的
    error[0].textContent = "账号或密码错误!";
    error.get(0).text = "账号或密码错误!";
    error.get(0).innerText = "账号或密码错误!";

    $("#info").html("<h1>显示一级标题</h1>");

    // 为空, 就是获取!
    console.log(error.text());
    console.log($("#info").html());
})
</script>
```

<div style="margin-top: 80px;">

---
</div>

## 操作元素属性

```js
$(选择器|DOM元素).val();              // 获取标签的value属性值
$(选择器|DOM元素).val(value值);       // 设置标签的value属性值

$(选择器|DOM元素).attr(属性名);        // 获取标签上给定属性名对应的属性值
$(选择器|DOM元素).attr(属性名, 属性值); // 设置标签上给定属性名对应的属性值

$(选择器|DOM元素).removeAttr(属性名);  // 移除标签上给定的属性名
```

示例
```html
<body>
    <input type="text" value="123">
    <div id="content" data-info="这是测试内容"></div>
</body>
<script src="./js/jquery-3.7.1.js"></script>
<script>
$(() => {
    // 获取input标签上的type属性值
    let type = $("input").attr("type");
    console.log(type);
    let value = $("input").attr("value");
    console.log(value);
    value = $("input").val(); // 只能获取value属性的值
    console.log(value);
    // $("input").attr("value", "234");
    $("input").val("678"); // 只能设置value属性的值
    // attr还可以获取到自定义的属性值
    let dateInfo = $("#content").attr("data-info");
    console.log(dateInfo);
    $("#content").attr("data-info", "这是修改的测试内容");
    // removeAttr方法可以移除元素的属性,包括自定义属性
    $("#content").removeAttr("data-info");
})
</script>
```

<div style="margin-top: 80px;">

---
</div>

## 操作元素样式
### 操作元素宽度和高度
```js
$(选择器|DOM元素).width();       // 获取标签的宽度
$(选择器|DOM元素).width(宽度值);  // 设置标签的宽度

$(选择器|DOM元素).height();      // 获取标签的高度
$(选择器|DOM元素).height(宽度值); // 设置标签的高度
```

示例
```html
<body>
    <div class="info"></div>
</body>
<script src="./js/jquery-3.7.1.js"></script>
<script type="text/javascript">
    $(function () {
        let info = $(".info");
        info.width('200px');
        info.height('200px');
        console.log(info.width() + " x " + info.height());
    })
</script>
```

<div style="margin-top: 80px;">

---
</div>

### 操作元素行内样式
```js
$(选择器|DOM元素).css(样式属性名);            // 获取标签上给定样式属性名对应的样式属性值
$(选择器|DOM元素).css(样式属性名, 样式属性值); // 设置标签上给定样式属性名对应的样式属性值

$(选择器|DOM元素).css({
    // css样式， 样式属性名中存在中横线时，使用驼峰命名法对样式属性名进行组装
});
```

示例
```html
<body>
    <p class="hx">这个是测试文字</p>
</body>
<script src="./js/jquery-3.7.1.js"></script>
<script>
$(() => {
    let e = $(".hx");
    e.css("backgroundColor", "red"); // 可以对原本'-'变成驼峰命名法
    console.log(e.css("background-color"));

    e.css({
        "color": "yellow",
        "font-size": "30px"
    });
})
</script>
```

<div style="margin-top: 80px;">

---
</div>

## 操作类样式
```js
$(选择器|DOM元素).addClass(类名);    // 为标签添加类样式
$(选择器|DOM元素).removeClass(类名); // 将给定的类样式从标签上移除

// 如果元素上存在给定的类样式，则移除改类样式；否则，则为标签添加上该类样式；
$(选择器|DOM元素).toggleClass(类名);

$(选择器|DOM元素).hasClass(类名);    // 判断标签上是否存在给定的类样式
```

示例
```html
<body>
    <p class="hx">这个是测试文字</p>
</body>
<script src="./js/jquery-3.7.1.js"></script>
<script>
$(() => {
    let e = $(".hx");
    e.addClass("info");
    console.log(e.hasClass("info"));

    e.removeClass("info");
    console.log(e.hasClass("info"));

    e.toggleClass("info");
    console.log(e.hasClass("info"));
})
</script>
```

<div style="margin-top: 80px;">

---
</div>

## 操作元素事件
| 名称 | 说明 |
| --- | --- |
| `click` | 鼠标左键单击元素 |
| `blur` | 元素失去焦点 |
| `focus` | 元素获得焦点 |
| `keydown` | 键盘按键被按下 |
| `keyup` | 键盘按键被按下后释放 |
| `keypress` |键盘按键按下不论释放与否都生效|
| `mouseover` | 鼠标移动至元素上 |
| `mouseout` | 鼠标移动至元素外 |
| `change` | (失去焦点或回车)元素的内容发生改变 |
| `input` | (一更新就)元素的内容发生改变 |

```js
// 为元素添加事件
$(选择器|DOM元素).on(事件名, 函数);
// 为元素添加事件
$(选择器|DOM元素).事件名(函数);
// 为元素中的子元素添加事件,通常用刷新部分的内容
$(选择器|DOM元素).on(事件名, 选择器, 函数);
```

示例
```html
<body>
    <p class="hx">点击我呀</p>
    <p id="btn">查询</p>
    <table>
        <thead>
            <tr>
                <td>姓名</td>
                <td>性别</td>
                <td>年龄</td>
                <td>操作</td>
            </tr>
        </thead>
        <tbody>
        </tbody>
    </table>
</body>
<script src="./js/jquery-3.7.1.js"></script>
<script type="text/javascript">
    $(function () {
        // 为ID为btn的元素添加点击事件
        $("#btn").click(function () {
            let students = [{
                name: '张三',
                sex: '男',
                age: 20
            }, {
                name: '李四',
                sex: '女',
                age: 20
            }];
            let html = "";
            for (let i = 0; i < students.length; i++) {
                html += "<tr>";
                html += "<td>" + students[i].name + "</td>";
                html += "<td>" + students[i].sex + "</td>";
                html += "<td>" + students[i].age + "</td>";
                html += '<td><a href="javascript:void(0)" class="update">修改</a><a href = "javascript:void(0)" class="delete"> 删除</a></td>';
                html += "</tr>";
            }
            $("tbody").html(html);
        });
        // 为tbody中使用的了类样式update的元素添加点击事件
        $("tbody").on("click", ".update", function () {
            alert("你点击了修改按钮")
        });
        // 为tbody中使用的了类样式delete的元素添加点击事件
        $("tbody").on("click", ".delete", function () {
            let result = confirm("确定要删除这条信息吗?");
            if (result) {
                console.log("删除了信息");
            }
        });
    })
</script>
<script>
$(() => {
    let e = $(".hx");
    e.on("click", () => alert("点我干嘛~"));
    e.mouseover(() => e.css("color", "red"));
    e.mouseout(() => e.css("color", "blueviolet"));
})
</script>
```