# 块级标签
## 块级标签特征
1. 总是在新行上开始

2. 高度，行高以及外边距和内边距都可控制

3. 宽度缺省是它的容器的100%

4. 可以容纳内联元素和其他块元素

## 标题标签

```html
<!--标题标签-->
<h1>一级标题</h1>
<h2>二级标题</h2>
<h3>三级标题</h3>
<h4>四级标题</h4>
<h5>五级标题</h5>
<h6>六级标题</h6>
```

### 效果

<h1>一级标题</h1>
<h2>二级标题</h2>
<h3>三级标题</h3>
<h4>四级标题</h4>
<h5>五级标题</h5>
<h6>六级标题</h6>

## 水平线标签

```html
<!--水平线标签-->
<hr>
```

### 效果

<hr>

## 段落标签

```html
<!--段落标签-->
<p>这个是段落标签</p>
```

### 效果
<p>这个是段落标签</p>


## 无序列表标签

```html
<!--无序列表标签-->
<ul>
    <li>列表项1</li>
    <li>列表项2</li>
    <li>列表项3</li>
    <li>列表项n</li>
</ul>
```

### 效果
<ul>
    <li>列表项1</li>
    <li>列表项2</li>
    <li>列表项3</li>
    <li>列表项n</li>
</ul>

## 有序列表标签

```html
<!--表有序列表标签-->
<ol type="A" start="1"> <!--写start可以指定从什么时候开始, 写type可以指定序号的样式-->
    <li>列表项1</li>
    <ol type="I" start="3900">
        <li>子列表项1</li>
        <li>子列表项2</li>
        <li>子列表项3</li>
    </ol>
    <li>列表项2</li>
    <li>列表项3</li>
    <li>列表项n</li>
</ol>
```

### 效果
<ol type="A" start="1"> <!--写start可以指定从什么时候开始, 写type可以指定序号的样式-->
    <li>列表项1</li>
    <ol type="I" start="3900">
        <li>子列表项1</li>
        <li>子列表项2</li>
        <li>子列表项3</li>
    </ol>
    <li>列表项2</li>
    <li>列表项3</li>
    <li>列表项n</li>
</ol>

## 表格标签

```html
<!--表格标签-->
<table border="2"><!--border表示单元格边框大小-->
    <caption>表格的标题</caption> <!--表格的标题-->
    
        <thead><!--表格的头部-->
            <tr><!--表格头部中的行-->
                <th>列名1</th><!--表格头部中的列-->
                <th>列名2</th>
                <th>列名n</th>
            </tr>
        </thead>

    <tbody><!--表格的主体部分-->
        <tr><!--表格主体部分中的行-->
            <td>列1的值</td><!--表格主体部分中的列-->
            <td>列2的值</td>
            <td>列n的值</td>
        </tr>

        <tr>
            <td>列1的值</td>
            <td>列2的值</td>
            <td>列n的值</td>
        </tr>

        <tr>
            <td>列1的值</td>
            <td>列2的值</td>
            <td>列n的值</td>
        </tr>
    </tbody>

    <tfoot><!--表格的尾部-->
        <tr><!--表格尾部中的行，主要用于信息统计-->
            <td>统计项名称</td><!--表格尾部中的列-->
            <td>列1的值</td>
            <td>列n的值</td>
        </tr>
    </tfoot>

</table>
```

### 效果1 (正表格)
<table border="2"><!--border表示单元格边框大小-->
    <caption>表格的标题</caption> <!--表格的标题-->   
        <thead><!--表格的头部-->
            <tr><!--表格头部中的行-->
                <th>列名1</th><!--表格头部中的列-->
                <th>列名2</th>
                <th>列名n</th>
            </tr>
        </thead>
    <tbody><!--表格的主体部分-->
        <tr><!--表格主体部分中的行-->
            <td>列1的值</td><!--表格主体部分中的列-->
            <td>列2的值</td>
            <td>列n的值</td>
        </tr>
        <tr>
            <td>列1的值</td>
            <td>列2的值</td>
            <td>列n的值</td>
        </tr>
        <tr>
            <td>列1的值</td>
            <td>列2的值</td>
            <td>列n的值</td>
        </tr>
    </tbody>
    <tfoot><!--表格的尾部-->
        <tr><!--表格尾部中的行，主要用于信息统计-->
            <td>统计项名称</td><!--表格尾部中的列-->
            <td>列1的值</td>
            <td>列n的值</td>
        </tr>
    </tfoot>
</table>

### 效果2 (杂表格)
<table cellpadding="0" cellspacing="0">
  <!-- 项目基本信息 -->
  <tbody>
    <tr rowspan="6" class="table-row">
      <th rowspan="5" class="table-name">项目基本信息</th>
      <td rowspan="1" class="table-first">项目基本概况</td>
      <td rowspan="1" class="table-all">-9-</td>
      <td rowspan="1" class="table-score">-9-</td>
      <td rowspan="6" class="table-get">83</td>
      <td rowspan="6" class="table-weight">0.8</td>
      <td rowspan="6" class="table-weiscore">26.4</td>
    </tr>
    <tr class="table-row">
      <td class="table-first">团队基本概况</td>
      <td class="table-all">9</td>
      <td class="table-score">9</td>
    </tr>
    <tr class="table-row">
      <td class="table-first">资本市场认可度</td>
      <td class="table-all">9</td>
      <td class="table-score">9</td>
    </tr>
    <tr class="table-row">
      <td class="table-first">项目宣传推广力度</td>
      <td class="table-all">9</td>
      <td class="table-score">9</td>
    </tr>
    <tr class="table-row">
      <td class="table-first">项目热度</td>
      <td class="table-all">9</td>
      <td class="table-score">9</td>
    </tr>
    <tr class="table-lastrow">
      <td class="table-null"></td>
      <td class="table-first">小计</td>
      <td class="table-all">9</td>
      <td class="table-score">9</td>
    </tr>
  </tbody>
  <!-- 项目团队评估 -->
  <tbody>
    <tr rowspan="6" class="table-row">
      <th rowspan="3" class="table-name">项目团队评估</th>
      <td rowspan="1" class="table-first">创始人背景信息</td>
      <td rowspan="1" class="table-all">-9-</td>
      <td rowspan="1" class="table-score">-9-</td>
      <td rowspan="4" class="table-get">83</td>
      <td rowspan="4" class="table-weight">0.8</td>
      <td rowspan="4" class="table-weiscore">26.4</td>
    </tr>
    <tr class="table-row">
      <td class="table-first">核心管理团队信息</td>
      <td class="table-all">9</td>
      <td class="table-score">9</td>
    </tr>
    <tr class="table-row">
      <td class="table-first">核心开发团队信息</td>
      <td class="table-all">9</td>
      <td class="table-score">9</td>
    </tr>
    <tr class="table-lastrow">
      <td class="table-null"></td>
      <td class="table-first">小计</td>
      <td class="table-all">9</td>
      <td class="table-score">9</td>
    </tr>
  </tbody>
  <!-- 项目方案评估 -->
  <tbody>
    <tr rowspan="6" class="table-row">
      <th rowspan="3" class="table-name">项目方案评估</th>
      <td rowspan="1" class="table-first">创始人背景信息</td>
      <td rowspan="1" class="table-all">-9-</td>
      <td rowspan="1" class="table-score">-9-</td>
      <td rowspan="4" class="table-get">83</td>
      <td rowspan="4" class="table-weight">0.8</td>
      <td rowspan="4" class="table-weiscore">26.4</td>
    </tr>
    <tr class="table-row">
      <td class="table-first">核心管理团队信息</td>
      <td class="table-all">9</td>
      <td class="table-score">9</td>
    </tr>
    <tr class="table-row">
      <td class="table-first">核心开发团队信息</td>
      <td class="table-all">9</td>
      <td class="table-score">9</td>
    </tr>
    <tr class="table-lastrow">
      <td class="table-null"></td>
      <td class="table-first">小计</td>
      <td class="table-all">9</td>
      <td class="table-score">9</td>
    </tr>
  </tbody>
  <!-- 合计 -->
  <tbody>
    <tr class="addtotal">
      <td colspan="2">合计</td>
      <td>240</td>
      <td colspan="3">240</td>
      <td>240</td>
    </tr>
    <tr class="totalscore">
      <td colspan="6">项目得分</td>
      <td>240</td>
    </tr>
  </tbody>
</table>

## 层标签

```html
<div>
  <!-- 内容 -->
</div>

<!-- 示例 -->
<div>第一行</div>
<div>第二行</div>
<div>第三行</div>
```

### 效果
<div>第一行</div>
<div>第二行</div>
<div>第三行</div>

## 表单

```html
<!--表单-->
<!--
    <form action="请求资源" method="请求方式">
        ...内容
    </form>
-->
<form action="b.html" method="get">
    <input type="b" value="提交"> <!-- 注: 这个 按钮是一个行级标签 -->
</form>
```

### 效果
<form action="b.html" method="get">
    <input type="b" value="提交">
</form>