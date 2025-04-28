# XML解析
## 解析方式
### DOM解析
DOM解析将XML文档一次性加载进内存，在内存中形成一颗dom树，优点是操作方便，可以对文档进行CRUD的所有操作，缺点就是占内存

### SAX解析
SAX解析式将XML文档逐行读取，是基于事件驱动的，优点是不占内存，缺点是只能读取，不能增删改对于XML操作，我们通常都是读取操作，增删改的情况较少，因此这里主要讲解SAX解析,SAX解析最优实现是`Dom4j`解析器。

*如果你是安卓, 那么可能要经常增删xml, 但是JavaWeb一般只是作为配置文件, 比如用于反射等, 所以只用读取*

# Dom4j 解析XML

导入`本地jar包`: [dom4j的下载与导入的流程](https://blog.csdn.net/faramita_of_mine/article/details/120791662)

```java
import org.dom4j.Attribute;
import org.dom4j.Document;
import org.dom4j.DocumentException;
import org.dom4j.Element;
import org.dom4j.io.SAXReader;

import java.io.File;
import java.util.Iterator;
import java.util.List;

public class Main {
    public static void main(String[] args) {
        // 构建一个SAX读取器
        SAXReader reader = new SAXReader();

        try {
            File file = new File("PlayXML/my.xml");

            System.out.println(file.getAbsolutePath());

            // ---解析开始---
            Document document = reader.read(file);

            // 获取根元素
            Element root = document.getRootElement();

            // 获取根元素的签名
            String rootName = root.getQualifiedName();
            System.out.println("根标签: " + rootName);

            // 遍历根元素
            List<Element> list = root.elements();

            for (Element it : list) {
                System.out.println("标签名: " + it.getQualifiedName());
                // 获取其所有属性
                List<Attribute> attributeList = it.attributes();
                for (Attribute a : attributeList) {
                    System.out.println("属性: " + a.getName() + " => " + a.getValue());
                }
            }

            System.out.println("==============");

            // 使用迭代器遍历
            Iterator<Element> iterator = root.elementIterator();
            while (iterator.hasNext()) {
                Element element = iterator.next();

                // 获取标签名
                System.out.println("标签名称: "
                        + element.getQualifiedName()
                        + "\tid: "
                        + element.attributeValue("id")); // 可以获取指定属性
            }
        } catch (DocumentException e) {
            throw new RuntimeException(e);
        }
    }
}
```
