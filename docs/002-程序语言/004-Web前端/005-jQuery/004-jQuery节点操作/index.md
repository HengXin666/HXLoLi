# jQuery 节点操作
## 创建、获取节点
```js
$(选择器);   // 获取节点
$(DOM元素);  // 将DOM元素转换为jQuery对象，该对象就是一个元素节点
$(HTML内容); // 将HTML内容转换为一个jQuery对象，该对象就是一个元素节点
```

示例
```html
<body>
    <p class="hx">获取我呀</p>
</body>
<script src="./js/jquery-3.7.1.js"></script>
<script>
    $(() => {
        let dom = document.getElementsByClassName("hx")[0];
        $(dom); // 这就是一个jQuery获取节点的对象

        $("#text");
        $("<a></a>"); // 这就是创建了元素节点
    });
</script>
```

<div style="margin-top: 80px;">

---
</div>

## 插入节点
```js
// 将给定的元素节点添加到jQuey选择器选择的标签内容的末尾
$(选择器|DOM元素).append(元素节点);
// 将jQuey选择器选择的标签添加到给定的元素节点内容的末尾
$(选择器|DOM元素).appendTo(元素节点);

// 将给定的元素节点添加到jQuey选择器选择的标签内容的最前面
$(选择器|DOM元素).prepend(元素节点);
// 将jQuey选择器选择的标签添加到给定的元素节点内容的最前面
$(选择器|DOM元素).prependTo(元素节点);

// 将给定的元素节点插入到jQuey选择器选择的标签后面
$(选择器|DOM元素).after(元素节点);
// 将jQuey选择器选择的标签添加到给定的元素节点前面
$(选择器|DOM元素).insertAfter(元素节点);

// 将给定的元素节点添加到jQuey选择器选择的标签前面
$(选择器|DOM元素).before(元素节点);
// 将jQuey选择器选择的标签添加到给定的元素节点前面
$(选择器|DOM元素).insertBefore(元素节点);
```

示例
```html
<body>
    <p class="hx">添加到我这</p>
</body>
<script src="./js/jquery-3.7.1.js"></script>
<script>
    $(() => {
        let dom = document.getElementsByClassName("hx")[0];
        // (下一级)内部头插/尾插
        $(dom).append($("<a href='#'>王德发1</a>"));
        $(dom).append($("<a href='#'>王德发2</a>"));
        $(dom).append($("<a href='#'>王德发3</a>"));
        $("<a href='#'>王德发4</a>").appendTo($(dom));
        $(dom).prepend($("<a href='#'>王德发5</a>"))
        $(dom).prepend($("<a href='#'>王德发6</a>"))
        
        // 同级尾插
        $(dom).after($("<a>qwq</a>"));
        $("<a>0.0</a>").insertAfter($(dom));

        // 同级头插
        $(dom).before($("<a>awa</a>"));
        $("<a>$v$</a>").insertBefore($(dom));
    });
</script>
```

<div style="margin-top: 80px;">

---
</div>

## 替换节点
```js
// 将jQuey选择器选择的标签添使用给定的元素节点替换
$(选择器|DOM元素).replaceWith(元素节点);
```

示例
```html
<body>
    <p class="hx">把我换掉?!</p>
</body>
<script src="./js/jquery-3.7.1.js"></script>
<script>
    $(() => {
        let dom = document.getElementsByClassName("hx")[0];
       $(dom).replaceWith($("<a>哈哈</a>"));
    });
</script>
```

<div style="margin-top: 80px;">

---
</div>

## 移除节点
```js
// 将jQuey选择器选择的标签中的所有内容清空
$(选择器|DOM元素).empty();

// 将jQuey选择器选择的标签（包括该标签中的所有内容）从DOM树中移除
$(选择器|DOM元素).remove();
```

示例
```html
<body>
    <p class="hx">直接删掉?!</p>
    <p class="hxx">直接删掉, 渣都没有?!</p>
</body>
<script src="./js/jquery-3.7.1.js"></script>
<script>
    $(() => {
        let dom = document.getElementsByClassName("hx")[0];
        $(dom).empty();
        $(".hxx").remove();
    });
</script>
```

<div style="margin-top: 80px;">

---
</div>

## 查找节点
```js
// 获取jQuey选择器选择的标签中的下一级子标签
$(选择器|DOM元素).children();

// 获取jQuey选择器选择的标签的父级标签
$(选择器|DOM元素).parent();

// 根据jQuey选择器1选择的标签，开始沿DOM树向上按选择器2查找，查找距离该元素最近的父级元素
$(选择器1|DOM元素).closest(选择器2);

// 获取jQuey选择器选择的标签紧邻匹配元素之后的元素
$(选择器1|DOM元素).next([选择器2]);

// 获取jQuey选择器选择的标签紧邻匹配元素之前的元素
$(选择器1|DOM元素).prev([选择器2]);

// 获取jQuey选择器选择的标签位于匹配元素前面和后面的所有同辈元素
$(选择器1|DOM元素).siblings([选择器2]);

// 在jQuey选择器选择的标签中查找拥有选择器2的元素
$(选择器1|DOM元素).find(选择器2);

// 获取jQuey选择器选择的标签中的第一个标签
$(选择器1|DOM元素).first();

// 获取jQuey选择器选择的标签中的最后一个标签
$(选择器1|DOM元素).last();
```

示例
```html
<body>
    <p>第一</p>
    <p class="hx">我是hx?!
        <a class="sadasfa">awa</a>
    </p>
    <p class="hxx">hxx是我?!</p>
    <p>最后</p>
</body>
<script src="./js/jquery-3.7.1.js"></script>
<script>
    $(() => {
        let dom = document.getElementsByClassName("hx")[0];
        console.log($(dom).children()); // 下一级子标签

        console.log($(dom).parent()); // 父标签

        // ... 略
    });
</script>
```

示例2
```html
<body>
    <div></div>
    <div class="content">
        <ul class="content">
            <li id="first">第一项</li>
            <li>第二项</li>
            <li>第三项</li>
            <li>第四项</li>
        </ul>
    </div>
</body>
    
<script src="./js/jquery-3.7.1.js"></script>
<script type="text/javascript">
    $(function () {
        // 定位ul,然后获取ul下一级所有子元素
        let children = $(".content > ul").children();
        console.log(children);
        let parent = $(".content > ul").parent(); // 获取ul的上一级父元素
        console.log(parent);
        // 查找距离该元素最近的父级元素
        let node = $("#first").closest(".content");
        console.log(node);
        // 查找与jQuery选择器定位的元素同级的下一个元素
        console.log($("#first").next());
        console.log($("div.content").next());
        console.log($("div.content").prev());
        // 获取与jQuery选择器定位的元素同级的所有元素
        console.log($("div.content").siblings());
        console.log($("li").first());
        console.log($("li").last());
        // 在jQuery选择器定位的元素中根据给定的选择器查找元素
        console.log($("div.content").find("#first"));
    })
</script>
```

<div style="margin-top: 80px;">

---
</div>

## 遍历节点
```js
$(选择器1|DOM元素).each(function(index, e) { //index表示元素下标，e表示元素
    // 遍历操作
});
```

示例
```html
<body>
    <p>第一</p>
    <p class="hx">我是hx?!
        <a class="sadasfa">awa</a>
        <p>子子子</p>
    </p>
    <p class="hxx">hxx是我?!</p>
    <p>最后</p>
</body>
<script src="./js/jquery-3.7.1.js"></script>
<script>
    $(() => {
        $("p").each((i, e) => {
            console.log(i);
            console.log(e);
        })
    });
</script>
```