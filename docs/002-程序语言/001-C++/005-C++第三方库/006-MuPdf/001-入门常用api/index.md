# 入门 MuPdf

- 网上找不到什么相关的教程... 只有官方文档和ai了...

## 0x00、安装

- 去官网, 下载rs版本, 或者去git仓库克隆最新分支
- 打卡 p/win32/mupdf.sln
- debug/rs 编译

最后只需要`libmupdf.lib`/`libresources.lib`/`libthirdparty.lib`这3个静态库. (好像有一个是debug没有的, 你拿rs的顶上)

- cmake可以如下配置

```cmake
include_directories("${PROJECT_SOURCE_DIR}/lib/mupdf/include")
if (CMAKE_BUILD_TYPE STREQUAL "Release")
    target_link_libraries(${projectName} PRIVATE "${PROJECT_SOURCE_DIR}/lib/mupdf/lib/Release/libmupdf.lib")
    target_link_libraries(${projectName} PRIVATE "${PROJECT_SOURCE_DIR}/lib/mupdf/lib/Release/libthirdparty.lib")
    target_link_libraries(${projectName} PRIVATE "${PROJECT_SOURCE_DIR}/lib/mupdf/lib/Release/libresources.lib")
else()
    target_link_libraries(${projectName} PRIVATE "${PROJECT_SOURCE_DIR}/lib/mupdf/lib/Debug/libmupdf.lib")
    target_link_libraries(${projectName} PRIVATE "${PROJECT_SOURCE_DIR}/lib/mupdf/lib/Debug/libthirdparty.lib")
    target_link_libraries(${projectName} PRIVATE "${PROJECT_SOURCE_DIR}/lib/mupdf/lib/Debug/libresources.lib")
endif()
```

## 0x01、常用api
以下是根据您的要求重新整理的详细 API 说明，包含完整注释规范和 `fz_try` 的使用场景说明：

---

### 一、核心 API 详细说明（带注释规范）

```cpp
/**
 * @brief 从自定义流打开 PDF 文档 (需自行管理流生命周期)
 * @param ctx        MuPDF 上下文（必须通过 fz_new_context 创建）
 * @param fmt        文档格式，如 "pdf"、"xps" 等
 * @param stream     自定义输入流（通过 fz_open_* 系列函数创建）
 * @return           文档对象指针，需用 fz_drop_document 释放
 * @warning 必须在 fz_try 块内调用，否则可能无法捕获异常
 */
fz_document* fz_open_document_with_stream(fz_context* ctx, const char* fmt, fz_stream* stream);

/**
 * @brief 创建基于内存数据的流对象（适用于 Qt 的 QBuffer 等内存流）
 * @param ctx        MuPDF 上下文
 * @param data       原始二进制数据指针
 * @param len        数据长度（字节）
 * @return           流对象指针，需用 fz_drop_stream 释放
 * @note 适合将 QByteArray 数据直接传递给 MuPDF
 */
fz_stream* fz_open_memory(fz_context* ctx, const unsigned char* data, size_t len);

/**
 * @brief 渲染页面到像素图（最终转换为 QImage）
 * @param ctx        MuPDF 上下文
 * @param page       页面对象（通过 fz_load_page 获取）
 * @param transform  变换矩阵（缩放/旋转等）
 * @param colorspace 颜色空间（通常用 fz_device_rgb）
 * @param alpha      是否启用透明通道（0 为不启用）
 * @return           像素图对象指针，需用 fz_drop_pixmap 释放
 * @code
 * fz_matrix matrix = fz_scale(2.0f, 2.0f); // 200% 缩放
 * fz_pixmap* pix = fz_new_pixmap_from_page(ctx, page, matrix, fz_device_rgb(ctx), 0);
 * @endcode
 */
fz_pixmap* fz_new_pixmap_from_page(fz_context* ctx, fz_page* page, fz_matrix transform, fz_colorspace* colorspace, int alpha);

/**
 * @brief 获取页面文本内容（用于实现复制功能）
 * @param ctx        MuPDF 上下文
 * @param page       页面对象
 * @return           文本页面结构指针，需用 fz_drop_stext_page 释放
 * @note 遍历 fz_stext_block 和 fz_stext_line 获取具体文本
 */
fz_stext_page* fz_new_stext_page_from_page(fz_context* ctx, fz_page* page);
```

---

### 二、`fz_try` 使用场景说明

#### **必须使用 `fz_try` 的情况**：
1. **所有文档/页面操作**：
   ```c
   fz_try(ctx) {
       doc = fz_open_document(ctx, "file.pdf");
       page = fz_load_page(ctx, doc, 0);
       pix = fz_new_pixmap_from_page(...);
   }
   fz_catch(ctx) {
       // 处理错误（如显示错误对话框）
   }
   ```

2. **涉及资源分配的操作**：
   ```c
   fz_stream* stm = nullptr;
   fz_try(ctx) {
       stm = fz_open_file(ctx, "test.pdf");
       // 其他可能失败的操作...
   }
   fz_catch(ctx) {
       fz_drop_stream(ctx, stm); // 必须手动清理已分配资源
   }
   ```

3. **嵌套调用场景**：
   ```c
   fz_try(ctx) {
       fz_try(ctx) {
           fz_run_page(ctx, page, dev, ...);
       }
       fz_catch(ctx) {
           // 处理子操作错误
       }
   }
   fz_catch(ctx) {
       // 处理外层错误
   }
   ```

#### **不需要 `fz_try` 的情况**：
1. **纯资源释放操作**：
   ```c
   // 这些函数本身不会抛出异常
   fz_drop_pixmap(ctx, pix);
   fz_drop_page(ctx, page);
   ```

2. **简单查询操作**：
   ```c
   int count = fz_count_pages(ctx, doc); // 无内存分配
   ```

---

### 三、典型使用示例（含完整错误处理）

```cpp
void renderToQImage(fz_context* ctx, const char* filename, QImage& outImage) {
    fz_document* doc = nullptr;
    fz_page* page = nullptr;
    fz_pixmap* pix = nullptr;
    
    fz_try(ctx) {
        // [1] 打开文档
        doc = fz_open_document(ctx, filename);
        
        // [2] 加载第一页
        page = fz_load_page(ctx, doc, 0);
        
        // [3] 创建缩放矩阵
        fz_matrix matrix = fz_scale(1.5f, 1.5f);
        
        // [4] 渲染到 pixmap
        pix = fz_new_pixmap_from_page(ctx, page, matrix, fz_device_rgb(ctx), 0);
        
        // [5] 转换为 QImage
        outImage = QImage(pix->samples, pix->w, pix->h, QImage::Format_RGB888);
    }
    fz_always(ctx) {
        // 无论是否异常都会执行的清理代码
        fz_drop_pixmap(ctx, pix);
        fz_drop_page(ctx, page);
        fz_drop_document(ctx, doc);
    }
    fz_catch(ctx) {
        qFatal("PDF Render Error: %s", fz_caught_message(ctx));
    }
}
```

---

### 四、关键内存管理规则

| 对象类型          | 创建函数                    | 释放函数                 | 生命周期示例                |
|-------------------|----------------------------|--------------------------|-----------------------------|
| `fz_context`      | `fz_new_context`           | `fz_drop_context`        | 整个应用生命周期            |
| `fz_document`     | `fz_open_document`         | `fz_drop_document`       | 从打开到关闭文档期间        |
| `fz_page`         | `fz_load_page`             | `fz_drop_page`           | 仅在页面渲染期间            |
| `fz_stream`       | `fz_open_file`             | `fz_drop_stream`         | 流读取操作期间              |
| `fz_pixmap`       | `fz_new_pixmap`            | `fz_drop_pixmap`         | 渲染完成到 QImage 拷贝后     |

---

### 五、高级技巧：带边界的预加载实现

```cpp
/**
 * @brief 预加载当前可视区域 (带边界缓冲)
 * @param visibleStart  可视区域起始页码
 * @param visibleEnd    可视区域结束页码
 * @param margin        预加载边界页数（默认前后各2页）
 */
void PDFRenderer::preloadPages(int visibleStart, int visibleEnd, int margin = 2) {
    int loadStart = qMax(0, visibleStart - margin);
    int loadEnd = qMin(m_totalPages-1, visibleEnd + margin);
    
    // 释放超出范围的缓存
    for(auto it = m_pageCache.begin(); it != m_pageCache.end();) {
        if(it.key() < loadStart || it.key() > loadEnd) {
            fz_drop_page(ctx, it.value());
            it = m_pageCache.erase(it);
        } else {
            ++it;
        }
    }
    
    // 加载新范围页面
    fz_try(ctx) {
        for(int i = loadStart; i <= loadEnd; ++i) {
            if(!m_pageCache.contains(i)) {
                fz_page* page = fz_load_page(ctx, m_doc, i);
                m_pageCache.insert(i, page);
            }
        }
    }
    fz_catch(ctx) {
        qWarning("预加载失败: %s", fz_caught_message(ctx));
    }
}
```

以上内容是dc生成的.

---

Fitz.h 提供的导出函数中，下列函数在渲染 PDF 文档时是必须使用的。

1. fz_new_context：创建渲染文档时的上下文变量。
2. fz_free_context：释放上下文变量所占用的资源。
3. fz_open_file_w：打开文件流（传入的文件名变量为 Unicode）
6. fz_open_document_with_stream：打开文件流对应的文档（PDF 或其它支持的文件格式）。
6. fz_close_document：关闭文档。
6. fz_close：关闭文件流。
6. fz_count_pages：获得文档的页数。
6. fz_load_page：加载文档指定的页面。
6. fz_free_page：释放文档页面占用的资源。
6. fz_bound_page：确定文档页面的尺寸。
6. fz_new_pixmap：创建渲染页面所用的图形画纸。
6. fz_clear_pixmap_with_value：清除画纸（通常用于将 PDF 文档的背景色设置为纯白色）。
6. fz_new_draw_device：从画纸创建绘图设备。
6. fz_find_device_colorspace：获取渲染页面所用的颜色域（彩色或灰色）。
6. fz_run_page：将页面渲染到指定的设备上。
6. fz_free_device：释放设备所占用的资源。
6. fz_drop_pixmap：释放画纸占用的资源。
6. fz_pixmap_samples：获取画纸的数据（用于将已渲染的画纸内容转换为 Bitmap）。

- https://www.cnblogs.com/pdfpatcher/archive/2012/11/25/2785154.html

---

## 0x02、自定义解析流

```C++
#include <fstream>

#include <QDebug>
#include <mupdf/pdf.h>

struct StreamState {
    explicit StreamState(const char* filePath)
        : _in(std::ifstream{filePath, std::ios::in | std::ios::binary})
    {}

    bool isOpen() const {
        return _in.is_open();
    }

    // 返回实际读取到的字节数
    int read(size_t max) {
        return _in.read((char *)buf, max).gcount();
    }

    bool isEof() const {
        return _in.eof();
    }

    auto pos() {
        return _in.tellg();
    }

    void seekg(int64_t offset, int whence) {
        _in.seekg(offset, whence);
    }

    inline static constexpr std::size_t MaxBufSize = 4096;

    unsigned char buf[MaxBufSize]{};

    ~StreamState() noexcept {
        _in.close();
    }

    std::ifstream _in;
};

/**
 * @brief 读取数据
 * @param ctx 
 * @param stm [in, out] 左闭右开: [rp, wp) 之间的数据是有效的, 如果有数据, 会更新 [rp, wq) 指向新的数据, 并且返回 *stm->rp++
 * @param max 期望读取的最大字节数
 * @return int 没有数据则返回-1, 否则返回读取到的字节数
 */
int streamNext(fz_context* ctx, fz_stream* stm, size_t max) {
    auto* sp = (StreamState*)stm->state;
    int res = sp->read(std::min(max, StreamState::MaxBufSize));
    if (res == 0 && sp->isEof()) {
        return -1;
    }
    stm->rp = &sp->buf[0];
    stm->wp = &sp->buf[res];
    stm->pos += res;
    return *stm->rp++;
}

/**
 * @brief 随机偏移
 * @param ctx 
 * @param stm 
 * @param offset 偏移量
 * @param whence 偏移的基准位置
 */
void streamSeek(fz_context* ctx, fz_stream* stm, int64_t offset, int whence) {
    auto* sp = (StreamState*)stm->state;
    // 根据 whence 转换为正确的 std::ios_base::seekdir
    std::ios_base::seekdir dir;
    switch (whence) {
        case 0: dir = std::ios::beg; break;
        case 1: dir = std::ios::cur; break;
        case 2: dir = std::ios::end; break;
        default: dir = std::ios::beg; break;
    }
    sp->seekg(offset, static_cast<int>(dir));
    // 重新获取当前的绝对位置, 并更新 stm->pos
    stm->pos = sp->pos();
}

/**
 * @brief 清理流的内部状态
 * @param ctx 
 * @param state 
 */
void streamDrop(fz_context* ctx, void* state) {
    // 无需操作, 因为RAII
    qDebug() << "raii";
}

struct MupdfRaii {
    explicit MupdfRaii(const char* filePath)
        : _ctx(fz_new_context(nullptr, nullptr, FZ_STORE_DEFAULT))
        , _ss(filePath)
        , _stream(fz_new_stream(_ctx, &_ss, streamNext, streamDrop))
    {
        // 库要求, pdf解析需要支持随机访问
        _stream->seek = streamSeek;

        // 注册文档处理程序
        fz_register_document_handlers(_ctx);

        // test
        fz_try(_ctx) {
            _doc = fz_open_document_with_stream(_ctx, filePath, _stream);
            int page_count = fz_count_pages(_ctx, _doc);
            qDebug() << "页数:" << page_count;
            fz_drop_document(_ctx, _doc);
        }
        fz_catch(_ctx) {
            qDebug() << "错误:" << fz_caught_message(_ctx);
        }
    }

    ~MupdfRaii() noexcept {
        fz_drop_document(_ctx, _doc);
        fz_drop_stream(_ctx, _stream);
        fz_drop_context(_ctx);
    }

    fz_context* _ctx;
    StreamState _ss;
    fz_stream* _stream;
    fz_document* _doc;
};

int main() {
    const char* filename1 = "D:/command/Github/HX-PDF-App/TestPdfSrc/C++-Templates-The-Complete-Guide-zh-20220903.pdf";
    const char* filename2 = "D:/command/Github/HX-PDF-App/TestPdfSrc/imouto.epub";
    MupdfRaii pdf1{filename1};
    MupdfRaii pdf2{filename2};
}
```

以上内容是根据源码的注释自己写的, 因为gpt/dc写几十次都不行...