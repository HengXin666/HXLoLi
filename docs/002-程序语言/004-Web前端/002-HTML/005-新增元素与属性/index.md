# HTML5 新增元素
## 结构标签

```html
<!DOCTYPE html>
<html>

<head>
    <meta charset="UTF-8">
    <title>结构标签</title>
    <style>
        html,
        body {
            width: 100%;
            height: 100%;
            margin: 0;
            padding: 0;
        }

        header,
        footer {
            height: 40px;
            background-color: black;
            color: white;
        }

        main {
            height: calc(100% - 40px);
            display: grid;
            grid-template-columns: 200px calc(100% - 200px);
        }

        aside {
            background-color: brown;
        }

        section {
            background-color: red;
            display: grid;
            grid-template-rows: 40px calc(100% - 80px) 40px;
        }

        nav {
            background-color: rebeccapurple;
        }
    </style>
</head>

<body>
    <header>页面头部 - header</header>
    
    <main>
        <aside>侧边栏 - aside</aside>
        <section>
            <nav>操作导航 - nav</nav>
            <article>正文 - article</article>
            <footer>底部 - footer</footer>
        </section>
    </main>
</body>

</html>
```

## 其他标签
### 音频标签 audio

```html
<!-- 
controls    属性控制页面上是否显示音频的操作控件
autoplay    属性表示音频在就绪后马上播放
loop        属性表示音频结束后重新播放
preload值：
    auto         当页面加载后载入整个音频
    metadata     当页面加载后只载入元数据
    none         当页面加载后不载入音频
-->
<audio src="D:\音乐\【P歌】アイデン貞貞メルトダウン（自我节操熔毁中）_ えなこ feat. P丸様。（别当欧尼酱了！O - 1.【P歌】アイデン貞貞メルトダウン（自我节操熔毁中）_ えなこ feat. P(Av607552731,P1).mp3" controls = "controls" autoplay="autoplay" loop="loop" preload="auto"></audio>
<!--音频标签还支持设置多个音频文件, 如果第一个不能播放就尝试第二个, 以此类推-->
<audio controls="controls" autoplay="autoplay" loop="loop" preload="metadata">
    <source src="音频路径1"/>
    <source src="音频路径2"/>
</audio>
```

### 视频标签 video

```html
<!--视频标签: 视频标签的用法与audio标签一样-->
<video src="D:\视频\001-二次元\【炮姐AMV】我永远都会守护在你的身边！ - 1.正片(Av810872,P1).mp4" controls="controls" autoplay="autoplay" loop="loop" preload="metadata">
</video>
```

### 列表标签

```html
<!--列表标签-->
<input list="id">
<datalist id="id">
    <option>选项1</option>
    <option>选项2</option>
    <option>选项3</option>
</datalist>
```

**示例:**

<input list="id">
<datalist id="id">
    <option>选项1</option>
    <option>选项2</option>
    <option>选项3</option>
</datalist>

### 时间与标记标签

```html
<!--时间与标记标签-->
<p>
    <!--时间标签没有什么实际意义，只是供机器识别：比如搜索引擎、爬虫分析-->
    我在<time datetime="2021-02-14">情人节</time>有个约会
</p>
<p>
    她长得很<mark>漂亮</mark>
</p>
```

### 进度标签 progress

```html
<!--进度标签-->
<progress value="66" max="100"></progress>
```

**示例:**

<progress value="66" max="100"></progress> 66.0721%

# HTML5 新增属性
## 全局属性

```html
<!--    全局属性
    元素是否允许可编辑内容          contentEditable
    是否必须对元素进行拼写或语法检查  spellcheck
    指定元素的tab键选择次序         tabindex
-->
<div style="height: 100px;" hidden></div>
<div style="height: 100px" contenteditable="true" spellcheck="true" tabindex="3"></div>
<div style="height: 100px" contenteditable="true" spellcheck="true" tabindex="2"></div>
<div style="height: 100px" contenteditable="true" spellcheck="true" tabindex="1"></div>
```

示例:

<div style="height: 100px;" hidden></div>
<div style="height: 100px" contenteditable="true" spellcheck="true" tabindex="3"></div>
<div style="height: 100px" contenteditable="true" spellcheck="true" tabindex="2"></div>
<div style="height: 100px" contenteditable="true" spellcheck="true" tabindex="1"></div>

## 表单属性
```html
<!--
    指定元素的默认提示信息            placeholder
    元素内容为必填                    required
    使用正则表达式检测元素内容是否合法  pattern
-->
<form action="" method="get">
    <input type="text" placeholder="请输入账号" required pattern="[a-z]{8,15}" title="账号只能为8到15位">
    <div>
        <button>注册</button>
    </div>
</form>

<form>
    <div>
      <label for="uname">选择一个用户名：</label>
      <input
        type="text"
        id="uname"
        name="name"
        required
        size="45"
        pattern="[a-z]{4,8}"
        title="4 到 8 个小写字母" />
      <span class="validity"></span>
      <p>用户名必须为小写字母，且长度为 4-8 个字符</p>
    </div>
    <div>
        <button>提交</button>
    </div>
</form>
```

示例:

<form action="" method="get">
    <input type="text" placeholder="请输入账号" required pattern="[a-z]{8,15}" title="账号只能为8到15位">
    <div>
        <button>注册</button>
    </div>
</form>

<form>
    <div>
      <label for="uname">选择一个用户名：</label>
      <input
        type="text"
        id="uname"
        name="name"
        required
        size="45"
        pattern="[a-z]{4,8}"
        title="4 到 8 个小写字母" />
      <span class="validity"></span>
      <p>用户名必须为小写字母，且长度为 4-8 个字符</p>
    </div>
    <div>
        <button>提交</button>
    </div>
</form>