# XML
## 什么是XML
XML全称是E<b style="color:red">x</b>tensible <b style="color:red">M</b>arkup <b style="color:red">L</b>anguage,可扩展标记语言

## XML语法
1. xml是一种文本文档，其后缀名为xml
2. xml文档内容的第一行必须是对整个文档类型的声明
3. xml文档内容中有且仅有一个根标签
4. xml文档内容中的标签必须严格闭合
5. xml文档内容中的标签属性值必须使用单引号或者双引号引起来
6. xml文档内容中的标签名区分大小写

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<lolis>
    <imouto>
        字符串
    </imouto>
    <loli name="名字" arg="13"></loli>
    <loli name="名称" arg="11" />
</lolis>
```

思考: 如果在XML文档内容中出现了像(<)这类似的特殊符号，怎么办？

<b style="color:red">使用标签CDATA来完成，CDATA标签中的内容会按原样展示</b>

语法:

```xml
<![CDATA[
    <!-- 内容 -->
]]>
```

XML文档可以自定义标签，为了更规范的使用XML文档，可以使用XML约束来限定XML文档中的标签使用。

XML约束可以通过`DTD`文档和`Schema`文档来实现。其中`DTD`文档比较简单，后缀名为`dtd`，而`Schema`技术则比较复杂，后缀名为`xsd`。