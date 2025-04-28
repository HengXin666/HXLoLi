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

# 文件上传下载
## 文件处理的包
`commons-io.jar`封装了常用的 IO 的相关操作，提供了`IOUtils`工具类供开发人员使用

`commons-fileupload.jar`文件上传的处理包，因为文件上传也会涉及到 IO 操作，因此，该包需要配合`commons-io.jar`使用。

- `FileItemFactory`文件项工厂，主要提供创建文件项的功能
- `DiskFileItemFactory`磁盘文件项工厂，主要用于解析上传文件时，创建对应的文件项
- `ServletFileUpload` `Servlet`文件上传对象，主要用于判断请求是否是文件上传请求，以及请求中的内容解析。解析时需要使用文件项工厂来创建文件项

## 文件上传
### form 表单上传

```html (jsp)
<form action="upload" method="post" enctype="multipart/form-data">
    <input type="file" name="file">
    <input type="submit" value="上传">
</form>
```

后端

```java
package com.HX.jsp.servlet;

import org.apache.commons.compress.utils.IOUtils;
import org.apache.commons.fileupload.FileItem;
import org.apache.commons.fileupload.FileItemFactory;
import org.apache.commons.fileupload.disk.DiskFileItemFactory;
import org.apache.commons.fileupload.servlet.ServletFileUpload;

import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import java.io.*;
import java.util.List;

@WebServlet(urlPatterns = "/upload")
public class UploadServet extends HttpServlet {
    private static final String SAVE_PATH = "D:\\command\\java_cmd\\qwqTomCat\\Upload";

    @Override
    protected void doPost(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
        if (!ServletFileUpload.isMultipartContent(req)) {
            throw new RuntimeException("未发现multipart/form-data"); // 没有上传文件
        }
        FileItemFactory factory = new DiskFileItemFactory(); // 磁盘文件项工厂，主要用于解析上传文件时，创建对应的文件项
        ServletFileUpload upload = new ServletFileUpload(factory); // 文件上传对象，主要用于判断请求是否是文件上传请求，以及请求中的内容解析
        // 文件上传的头部信息编码格式为UTF-8
        upload.setHeaderEncoding("UTF-8");
        // 文件上传的最大大小为50M
        upload.setSizeMax(1024 * 1024 * 50);
        // 单个文件最大大小为5M
        upload.setFileSizeMax(1024 * 1024 * 5);
        boolean success = true;
        try {
            // 解析请求，得到form表单上传的每一个与文件相关的项
            List<FileItem> fileItems = upload.parseRequest(req);

            for (FileItem item : fileItems) {
                if (item.isFormField()) { // 如果是表单字段
                    System.out.println(item.getName());
                } else {
                    File parent = new File(SAVE_PATH); // 使用相对路径会被保存到 tomcat 的 bin 里面
                    if (!parent.exists()) {
                        parent.mkdirs();
                    }
                    File file = new File(parent, item.getName());
                    InputStream is = item.getInputStream();
                    OutputStream os = new FileOutputStream(file);
                    // 使用commons-io包提供的工具类，将输入流中的信息拷贝至输出流，也就是写入文件
                    IOUtils.copy(item.getInputStream(), os);
                    // 关闭输入流
                    IOUtils.closeQuietly(is);
                    // 关闭输出流
                    IOUtils.closeQuietly(os);
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
            success = false;
        }
        resp.setCharacterEncoding("UTF-8");
        resp.setContentType("text/html;charset=utf-8");
        PrintWriter writer = resp.getWriter();
        writer.print(success ? "上传成功" : "上传失败");
        writer.flush();
        writer.close();
    }
}
```

### ajax 文件上传

```html (jsp)
<input type="file" id="file">
<input type="button" value="上传" id="uploadBtn">
<script type="text/javascript">
    $(() => {
        $("#uploadBtn").click(() => {
            let files = $("#file")[0].files;
            if(files && files.length > 0){
                // 模拟表单数据
                let formData = new FormData();
                formData.append("file", files[0]);
                $.ajax({
                    url: 'upload',
                    type: 'post',
                    data: formData,
                    processData: false, // 告诉jQuery不要处理数据
                    contentType: false, // 告诉jQuery不要设置内容的类型
                    success: function (resp) {
                        alert(resp);
                    },
                    error: function (error) {
                    }
                })
            } else {
                alert("请选择上传文件");
            }
        });
    });
</script>
```

## 文件下载

```html (jsp)
<a href="download?name=学习.png">下载: 学习.png</a>
```

后端: 1. 获取文件名, 2. 设置返回类型
```java
package com.HX.jsp.servlet;

import org.apache.commons.compress.utils.IOUtils;

import javax.servlet.ServletException;
import javax.servlet.annotation.WebServlet;
import javax.servlet.http.HttpServlet;
import javax.servlet.http.HttpServletRequest;
import javax.servlet.http.HttpServletResponse;
import java.io.*;
import java.nio.charset.StandardCharsets;

@WebServlet(urlPatterns = "/download")
public class DownloadServet extends HttpServlet {
    private static final String DOWNLOAD_PATE = "E:\\图片";

    @Override // 默认 <a> 是 get 请求
    protected void doGet(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
        String name = req.getParameter("name");              // 获取下载的文件名
        File file = new File(DOWNLOAD_PATE, name);           // 定位下载的文件
        byte[] data = name.getBytes(StandardCharsets.UTF_8); // 获取下载的文件名的字节数据
        // 转换编码格式，重新构建字符串，因为浏览器默认支持的编码是ISO_8859_1,因此，要转换为这种编码下中文才能正常显示
        name = new String(data, StandardCharsets.ISO_8859_1);
        // 设置内容处理方案: 以附件的形式处理
        resp.setHeader("Content-Disposition", "attachment;filename="+name);
        OutputStream os = resp.getOutputStream();
        InputStream is = new FileInputStream(file);
        IOUtils.copy(is, os);
        IOUtils.closeQuietly(is);
        IOUtils.closeQuietly(os);
    }
}
```

# Excel 处理
## `EasyExcel` 介绍
`EasyExcel`是阿里巴巴开源的一个 Excel 处理框架，以使用简单、节省内存著称。`EasyExcel`能大大减少占用内存的主要原因是在解析 Excel 时没有将文件数据一次性全部加载到内存中，而是从磁盘上一行行读取数据，逐个解析。

## `EasyExcel` 生成 Excel
存放学生数据类, 并且和Excel内容关联
```java
package com.HX.jsp.pojo;

import com.alibaba.excel.annotation.ExcelProperty;

public class ExcelStu {
    // ExcelProperty 表示在生成的Excel表格中的属性, value表示该属性对应的表头名称, index表示该属性所处的列的位置
    @ExcelProperty(value = "班级", index = 0)
    private String className;
    @ExcelProperty(value = "姓名", index = 1)
    private String name;
    @ExcelProperty(value = "分数", index = 2)
    private int fraction;
    // toString / get / set / 构造函数(记得要有无参构造, 下面没有会报错) 略
}    
```

写入
```java
package com.HX.jsp.excel;

import com.HX.jsp.pojo.ExcelStu;
import com.alibaba.excel.EasyExcel;

import java.util.ArrayList;
import java.util.List;

public class ExcelUtil {
    private static final String EXCEL_PATH = "E:/学生成绩.xlsx";

    public static void main(String[] args) {
        List<ExcelStu> stus = new ArrayList<>();
        for(int i = 1; i <= 10; ++i) { // 搞点假数据
            ExcelStu excelStu = new ExcelStu("计科" + i + "班", "张" + i, 100 + i);
            stus.add(excelStu);
        }
        EasyExcel.write(EXCEL_PATH, ExcelStu.class) // 保存的路径, 写入的类的类型
                .sheet("用户信息表") // 表名称
                .doWrite(stus); // 写入容器内容
    }
}
```

提取方法 + (外观模式)封装一下:

```java
public class ExcelUtil {
    private static final String EXCEL_PATH = "E:/学生成绩.xlsx";

    public static void main(String[] args) {
        List<ExcelStu> stus = new ArrayList<>();
        for(int i = 1; i <= 10; ++i) {
            ExcelStu excelStu = new ExcelStu("计科" + i + "班", "张" + i, 100 + i);
            stus.add(excelStu);
        }

        generateExcel(EXCEL_PATH, "学生成绩", ExcelStu.class, stus);
    }

    /**
     * 生成Excel表格 (如果路径相同, 则为覆盖原内容!)
     * @param path 生成的路径
     * @param sheetName 表(sheet)名称
     * @param clazz 存储的数据类型
     * @param data 存储的数据
     * @param <T>
     */
    public static<T> void generateExcel(String path, String sheetName, Class<T> clazz, List<T> data) {
        EasyExcel.write(path, clazz) // 保存的路径, 写入的类的类型
                .sheet(sheetName)    // 表名称
                .doWrite(data);      // 写入容器内容
    }
}
```

## `EasyExcel` 解析 Excel

```java
// 解析Excel
// 读取监听器, 每一行读取动作都会被监听
ReadListener<ExcelStu> readListener = new ReadListener<ExcelStu>() {
    @Override
    public void invoke(ExcelStu excelStu, AnalysisContext analysisContext) {
        System.out.println("读取了一行: " + excelStu.toString());
    }

    @Override
    public void doAfterAllAnalysed(AnalysisContext analysisContext) {
        System.out.println("读取结束!");
    }
};

// 读取给定的Excel, 读取的内容转换为ExcelStu类型的对象，读取时使用readListener进行监听
EasyExcel.read(EXCEL_PATH, ExcelStu.class, readListener)
        .sheet() // sheet()方法, 不指定名称, 则表示读取所有的sheet
        .doRead(); // doRead() 表示表示执行读取动作 [注: 需要ExcelStu有无参构造!]
```

封装一下

```java
// 解析Excel
List<ExcelStu> excelStuList = parseExcel(EXCEL_PATH, null, ExcelStu.class);

/**
 * 读取Excel表格
 * @param path 读取的路径
 * @param sheetName 读取的表(sheet)的名称, 读取全部则填 null
 * @param clazz 读取的数据的类型
 * @param <T>
 * @return 解析得到的数据
 */
public static<T> List<T> parseExcel(String path, String sheetName, Class<T> clazz) {
    List<T> res = new ArrayList<>();
    // 读取监听器, 每一行读取动作都会被监听
    ReadListener<T> readListener = new ReadListener<T>() {
        @Override
        public void invoke(T t, AnalysisContext analysisContext) {
            // System.out.println("读取了一行: " + t.toString());
            res.add(t);
        }

        @Override
        public void doAfterAllAnalysed(AnalysisContext analysisContext) {
            // System.out.println("读取结束!");
        }
    };

    // 读取给定的Excel, 读取的内容转换为ExcelStu类型的对象，读取时使用readListener进行监听
    EasyExcel.read(path, clazz, readListener)
            .sheet(sheetName) // sheet()方法, 不指定名称, 则表示读取所有的sheet
            .doRead(); // doRead() 表示表示执行读取动作
    return res;
}
```

## `EasyExcel` 导入导出
因为我们网络是使用io流来传输的, 所以上传和下载也应该使用流

所以还需要封装:
```java
public class ExcelUtil {
    private static final String EXCEL_PATH = "E:/学生成绩.xlsx";

    private static final int SHEET_MAX_SIZE = 3; // 单个表的最大内容量

    public static void main(String[] args) throws IOException {
        List<ExcelStu> stus = new ArrayList<>();
        for(int i = 1; i <= 10; ++i) {
            ExcelStu excelStu = new ExcelStu("计科" + i + "班", "张" + i, 100 + i);
            stus.add(excelStu);
        }

        writeExcel("成绩", new FileOutputStream(EXCEL_PATH), ExcelStu.class, stus);

        readExcel(new FileInputStream(new File(EXCEL_PATH)), ExcelStu.class);
    }

    /**
     * 读取输入流中的数据
     * @param is 输入流
     * @param clazz 类型
     * @return 读取到的数据的 ArrayList
     * @param <T>
     */
    public static<T> List<T> readExcel(InputStream is, Class<T> clazz) {
        List<T> res = new ArrayList<>();
        // 读取监听器, 每一行读取动作都会被监听
        ReadListener<T> readListener = new ReadListener<T>() {
            @Override
            public void invoke(T t, AnalysisContext analysisContext) {
                System.out.println("读取了一行: " + t.toString());
                res.add(t);
            }

            @Override
            public void doAfterAllAnalysed(AnalysisContext analysisContext) {
                System.out.println("读取一个sheet结束!");
            }
        };

        EasyExcel.read(is, readListener).doReadAll();

        return res;
    }

    /**
     * 将List的数据写入到输出流
     * @param sheetName 表名
     * @param os 输出流
     * @param clazz 保存数据的类型
     * @param data 数据
     * @param <T>
     */
    public static<T> void writeExcel(String sheetName, OutputStream os, Class<T> clazz, List<T> data) throws IOException {
        ExcelWriter excelWriter = EasyExcel.write(os, clazz).build();
        int sheet_count = data.size() / SHEET_MAX_SIZE + (data.size() % SHEET_MAX_SIZE == 0 ? 0 : 1); // 分表
        for (int i = 0; i < sheet_count; ++i) {
            WriteSheet sheet = new WriteSheet();
            sheet.setSheetNo(i);
            sheet.setSheetName(sheetName + (i + 1));
            int start = i * SHEET_MAX_SIZE;
            int end = (i + 1) * SHEET_MAX_SIZE;
            if (end > data.size())
                end = data.size();
            excelWriter.write(data.subList(start, end), sheet);
        }
        excelWriter.finish();
        os.close();
    }
}
```

修改为从服务器下载Excel (直接从之前下载文件的代码中进行修改了)

```java
@WebServlet(urlPatterns = "/download")
public class DownloadServet extends HttpServlet {
    private static final String DOWNLOAD_PATE = "E:\\";

    @Override // 默认 <a> 是 get 请求
    protected void doGet(HttpServletRequest req, HttpServletResponse resp) throws ServletException, IOException {
        String name = req.getParameter("name");           // 获取下载的文件名
        File file = new File(DOWNLOAD_PATE, name);          // 定位下载的文件
        byte[] data = name.getBytes(StandardCharsets.UTF_8); // 获取下载的文件名的字节数据
        // 转换编码格式，重新构建字符串，因为浏览器默认支持的编码是ISO_8859_1,因此，要转换为这种编码下中文才能正常显示
        name = new String(data, StandardCharsets.ISO_8859_1);
        // 设置内容处理方案: 以附件的形式处理
        resp.setHeader("Content-Disposition", "attachment;filename="+name);
        OutputStream os = resp.getOutputStream();

        // 应该是通过数据库操作得到 List (此处是模拟从数据库获取数据的过程)
        List<ExcelStu> stus = ExcelUtil.readExcel(new FileInputStream(file), ExcelStu.class);
        ExcelUtil.writeExcel("成绩", os, ExcelStu.class, stus);

//        InputStream is = new FileInputStream(file);
//        IOUtils.copy(is, os);
//        IOUtils.closeQuietly(is);
        IOUtils.closeQuietly(os);
    }
}
```