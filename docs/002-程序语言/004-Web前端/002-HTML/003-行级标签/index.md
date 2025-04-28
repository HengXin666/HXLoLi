# 行级标签
## 行级标签特征
1. 和其他元素都在一行上

2. 高度，行高及外边距和内边距不可改变

3. 宽度就是其内容的宽度，不可改变

## 图像标签

```html
<!--
    图像标签
    <img src="logo.png" title="鼠标放在上面显示的内容" alt="图片未加载时显示">
    height（高度） 与 width（宽度）属性用于设置图像的高度与宽度。
    属性值默认单位为像素
-->
<img src="D:\图片\88438977_p0.png" title="これはロリです" alt="WI-FIがない">
<img src="D:\图片\88438977_p0.png" alt="Pulpit rock" width="304" height="228">
```

## 范围标签

```html
<!--范围标签
    <span>内容</span>
-->
价格<span style="color: rgb(255, 24, 24);font-size: 30px;">19.9</span>元
```

## 超链接标签

```html
<!--超链接标签
    <a href="资源地址" target="目标窗口位置">内容</a>
    其中target常用如下：
        _blank：    在新窗口中打开
        _self：     在当前窗口中打开，是超链接target属性的默认值

    基本的注意事项 - 有用的提示
    请始终将正斜杠添加到子文件夹。
    假如这样书写链接：href="https://www.runoob.com/html"，就会向服务器产生两次 HTTP 请求。
    这是因为服务器会添加正斜杠到这个地址，然后创建一个新的请求，
    就像这样：href="https://www.runoob.com/html/"。
-->
<a href="www.baidu.com" target="_blank">去购买</a><br>

<!--超链接通常分为页面间链接、锚链接和功能性链接-->
<!--页面间链接: -->
<a href="页面名称">页面间</a>

<!--锚链接: 
    <a href="页面名称#元素的ID属性值">内容</a>
    // 同一个页面中, 页面名称可以省略
-->
<a id = "dibu" href="#dinbu">去顶部</a> <!--本html内跳转--><br>
<a href="b.html#content">去第二个页面</a><!--页面之间跳转--><br>

<!--
    功能性链接:
-->
<a href="mailTo:282000500@qq.com">联系我们</a>
<br><a href="document.pdf" download>下载文档</a>    <!--下载链接-->
```

**示例:**

<a href="mailTo:282000500@qq.com">联系我们</a>

## 输入标签

```html
<!--输入标签
    <input type="类型" name="名称" value="值">
-->
<div>账号:<input type="text" name = "username" maxlength="5"></div>
<div>密码:<input type="password" name="password"></div>
<div>生日:<input type="date" name="birthday"></div>
<div><del>更加精确的时间:</div><input type="datetime-local" name="114514"></div>
<div><i>性别</i>:
    <input type="radio" name="sex" value="0">男
    <input type="radio" name="sex" value="1">女
    <input type="radio" name="sex" value="2" checked>未知 <!--checked 是默认在那个单选框里面-->
</div>
<div>头像:<input type="file" name="touxian"></div>
<div><input type="button" value="登录"></div>
<input type="hidden" name="名称"> <!-- 隐藏域 这个不常用-->
```

**示例：**

<div>账号:<input type="text" name = "username" maxlength="5"></div>
<div>密码:<input type="password" name="password"></div>
<div>生日:<input type="date" name="birthday"></div>
<div><del>更加精确的时间:</div><input type="datetime-local" name="114514"></div>
<div><i>性别</i>:
    <input type="radio" name="sex" value="0">男
    <input type="radio" name="sex" value="1">女
    <input type="radio" name="sex" value="2" checked>未知 <!--checked 是默认在那个单选框里面-->
</div>
<div>头像:<input type="file" name="touxian"></div>
<div><input type="button" value="登录"></div>
<input type="hidden" name="名称"> <!-- 隐藏域 这个不常用-->

## 文本域

```html
<!--文本域-->
<textarea name="名称" placeholder="提示信息"></textarea>
```

**示例:**

<textarea name="名称" placeholder="提示信息"></textarea>

## 下拉列表框

```html
<!--下拉列表框-->
<select>
    <option value="值">请选择</option>
    <option value="值">114514</option>
    <option value="值">1433223</option>
    <option value="值">0721</option>
</select>

<!--只读和禁用-->
<input type="类型" name="名称" readonly> <!-- 只能读，不能修改 -->
<input type="类型" name="名称" disabled> <!-- 禁用 -->
<select name="名称" disabled> <!-- 禁用 -->
    <option value="值">显示值</option>
    <option value="值">显示值</option>
    <option value="值">显示值</option>
</select>
<textarea name="名称" placeholder="提示信息" readonly></textarea><!-- 只能读，不能修改 -->
<textarea name="名称" placeholder="提示信息" disabled></textarea><!-- 禁用 -->
```

**示例:**

<select>
    <option value="值">请选择</option>
    <option value="值">114514</option>
    <option value="值">1433223</option>
    <option value="值">0721</option>
</select>
<input type="类型" name="名称" readonly> <!-- 只能读，不能修改 -->
<input type="类型" name="名称" disabled> <!-- 禁用 -->
<select name="名称" disabled> <!-- 禁用 -->
    <option value="值">显示值</option>
    <option value="值">显示值</option>
    <option value="值">显示值</option>
</select>
<textarea name="名称" placeholder="提示信息" readonly></textarea><!-- 只能读，不能修改 -->
<textarea name="名称" placeholder="提示信息" disabled></textarea><!-- 禁用 -->

## 文本样式

```html
<!--文本居中样式-->
<p align="center"> 这个是一个居中的文字 </p>
<!--文本格式化标签
<b>     定义粗体文本
<em>    定义着重文字
<i>     定义斜体字
<small>    定义小号字
<strong>定义加重语气(好像也是加粗)
<sub>    定义下标字
<sup>    定义上标字
<ins>    定义插入字 (下划线)
<del>    定义删除字
-->
```

**示例:**

<p align="center"> 这个是一个居中的文字 </p>

<b>     定义粗体文本 </b>
<em>    定义着重文字 </em>
<i>     定义斜体字 </i> 
<small>    定义小号字 </small>
<strong>定义加重语气(好像也是加粗) </strong>
<sub>    定义下标字 </sub>
<sup>    定义上标字 </sup>
<ins>    定义插入字 (下划线) </ins>
<del>    定义删除字 <del>
