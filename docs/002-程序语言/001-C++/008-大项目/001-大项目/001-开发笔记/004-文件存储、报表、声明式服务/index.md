# 一、文件存储
## 1.1 先决条件: 安装FastDFS
- 按照 doc 分支 的环境搭建来即可
- 可能需要: 安装Nacos

## 1.2 大自然的馈赠: FastDfsClient.h
于`\zero-one-08mes\mes-cpp\lib-common\include\FastDfsClient.h`

是一个 **FastDFS文件上传下载客户端**(`FastDfsClient`类), 支持跨平台(Linux/Windows)

常用方法:

```C++
//************************************
// Method:    uploadFile
// FullName:  FastDfsClient::uploadFile
// Access:    public 
// Returns:   std::string 上传成功返回fastdfs存储的文件名，包含：组名+文件名，上传失败返回空字符串
// Description: 上传文件
// Parameter: const std::string & fileName 包含路径的上传文件名
//************************************
std::string uploadFile(const std::string& fileName);

//************************************
// Method:    uploadFile
// FullName:  FastDfsClient::uploadFile
// Access:    public 
// Returns:   std::string 上传成功返回fastdfs存储的文件名，包含：组名+文件名，上传失败返回空字符串
// Description: 上传文件
// Parameter: const char * buff 文件二进制字节数据
// Parameter: size_t size 文件大小
// Parameter: const std::string& extName 文件后缀名
//************************************
std::string uploadFile(const char* buff, size_t size, const std::string& extName = "");

//************************************
// Method:    downloadFile
// FullName:  FastDfsClient::downloadFile
// Access:    public 
// Returns:   std::string 返回包含路径的文件名
// Description: 下载文件
// Parameter: const std::string & fieldName fastdfs对应的文件名，包含：组名+文件名
// Parameter: std::string * savePath 保存文件根目录，要求字符串结尾不带//或\
//************************************
std::string downloadFile(const std::string& fieldName, std::string* savePath);

//************************************
// Method:    deleteFile
// FullName:  FastDfsClient::deleteFile
// Access:    public 
// Returns:   bool 删除成功返回true
// Description: 删除文件
// Parameter: const std::string & fieldName fastdfs对应的文件名，包含：组名+文件名
//************************************
bool deleteFile(const std::string& fieldName);
```

## 1.3 如何使用`FastDfs`
- `zero-one-08mes\mes-cpp\arch-tests\tests\fastdfs\test-fastdfs.cpp`中有示例

### 1.3.1 includeのご注意

```C++
#include ...
  
// 注意: 用到FastDfs的地方都要在最后在导入DFS的头文件
#include "FastDfsClient.h"
```

### 1.3.2 创建一个管理客户端的对象
从`zero-one-08mes\mes-cpp\arch-tests\tests\fastdfs\test-fastdfs.cpp`到`zero-one-08mes\mes-cpp\arch-demo\controller\file\FileController.h`

~~遇到了一个问题: 似乎无法获取到文件名? 导致无法保存? 手动设置文件名则可以保存!?~~ <-- あほか！三个配置文件就改了两个!!!


```C++
// 注意：用到FastDfs的地方都要在最后在导入DFS的头文件
#include "FastDfsClient.h"

// 定义一个临时变量用于存储上次上传成功后的文件field名称
std::string tmpField = "";

/**
 * 创建一个测试夹具，用于管理Dfs客户端对象创建过程
 */
class FastDfsTest : public testing::Test {
protected:
    void SetUp() override
    {
        // 定义一个Nacos客户端对象，用于获取配置
        NacosClient ns(TEST_NS_ADDR, TEST_NS_NAMESPACE);
#ifdef LINUX
        // 读取配置数据节点
        auto thirdServerConfig = ns.getConfig("third-services.yaml");
        // 从Nacos配置中心获取FastDFS客户端配置数据
        std::string config = ns.getConfigText("client.conf");
        // 定义客户端对象
        this->client = new FastDfsClient(config, false);
        // 设置一个文件上传地址
        this->filename = "/home/file/1.zip";
#else
        // 读取配置数据节点
        auto thirdServerConfig = ns.getConfig("./conf/third-services.yaml");
        // +FastDFS客户端配置数据
        std::string ipPort = YamlHelper().getString(&thirdServerConfig, "fastdfs.tracker-servers");
        std::string ip = ipPort.substr(0, ipPort.find(":"));
        int port = stoi(ipPort.substr(ipPort.find(":") + 1));
        // 设置客户端对象
        this->client = new FastDfsClient(ip, port);
        // 设置一个文件上传地址
        this->filename = "E:/loli.jpg";
#endif
        // 设置url前缀
        this->urlPrefix = "http://" + YamlHelper().getString(&thirdServerConfig, "fastdfs.nginx-servers") + "/";
    }
    void TearDown() override
    {
        if (client)
        {
            delete client;
            client = nullptr;
        }
    }
    // DFS客户端连接对象
    FastDfsClient* client = nullptr;
    // 文件下载地址前缀
    std::string urlPrefix = "";
    // 声明一个上传文件的文件名
    std::string filename = "";
};
```

注: 这个是一个粗糙的测试

### 1.3.3 测试文件上传

```C++
// 测试文件上传
TEST_F(FastDfsTest, Upload) {
    tmpField = client->uploadFile(filename);
    ASSERT_NE(tmpField, "");
    // 输出文件下载地址
    std::string downloadUrl = urlPrefix + tmpField;
    std::cout << "download url: " << downloadUrl << std::endl;
}
```

### 1.3.4 测试文件下载

```C++
// 测试文件下载
TEST_F(FastDfsTest, Download) {
    if (tmpField.empty())
        return;
    std::string path = "./public/fastdfs";
    std::string fullPath = client->downloadFile(tmpField, &path);
    ASSERT_NE(fullPath, "");
    std::cout << "download savepath is : " << fullPath << std::endl;
}
```

### 1.3.5 测试文件删除

```C++
// 测试文件删除
TEST_F(FastDfsTest, Delete) {
    if (tmpField.empty())
        return;
    bool res = client->deleteFile(tmpField);
    ASSERT_EQ(res, true);
}
```

---
以上只是让你熟悉一下文件的上传/下载/删除操作, 实际用户使用的时候是这样的

## 1.4 后端应用
### 1.4.1 定义端点

```C++
#include "domain/vo/BaseJsonVO.h"
#include "ApiHelper.h"
#include "ServerInfo.h"
#include "domain/vo/file/FileVO.h"

#include OATPP_CODEGEN_BEGIN(ApiController)

/**
 * 文件操作示例接口
 */
class FileController : public oatpp::web::server::api::ApiController
{
    // 定义控制器访问入口
    API_ACCESS_DECLARE(FileController);
public:
    // 定义一个单文件上传接口
    // 定义描述
    API_DEF_ENDPOINT_INFO(ZH_WORDS_GETTER("file.upload.summary"), uploadFile, StringJsonVO::Wrapper);
    // 定义端点
    API_HANDLER_ENDPOINT(API_M_POST, "/file/upload", uploadFile, REQUEST(std::shared_ptr<IncomingRequest>, request), execUploadOne(request));

    // ...
};
```

### 1.4.2 编写控制层

```C++
// FastDFS需要导入的头
#include "ServerInfo.h"
#include "NacosClient.h"
#include "FastDfsClient.h"
#include "SimpleDateTimeFormat.h"

StringJsonVO::Wrapper FileController::execUploadOne(std::shared_ptr<IncomingRequest> request)
{
    // 1 初始化
    API_MULTIPART_INIT(container, reader);

    // 2 配置读取器
    API_MULTIPART_CONFIG_MEMO_DEFAULT(reader, -1);
    //API_MULTIPART_CONFIG_MEMO(reader, "file", -1);
    //API_MULTIPART_CONFIG_MEMO(reader, "nickname", -1);
    //API_MULTIPART_CONFIG_MEMO(reader, "age", -1);

    // 3 读取数据
    request->transferBody(&reader);
    /* 打印上传总部分数 */
    OATPP_LOGD("Multipart", "parts_count=%d", container->count());

    // 4 解析数据
    /* TODO: 解析的数据具体逻辑，需要根据你的业务需求来，下面是使用示例而已。 */
    /* 获取表单数据 */
    API_MULTIPART_PARSE_FORM_FIELD_STR(container, "nickname", nickname);
    API_MULTIPART_PARSE_FORM_FIELD_NUM(container, "age", Int32, age, stoi);
    /* 打印表单数据 */
    if (nickname)
        OATPP_LOGD("Multipart", "nickname='%s'", nickname.getValue({}).c_str());
    if (age)
        OATPP_LOGD("Multipart", "age=%d", age.getValue({}));
    
    /* 获取文件数据 */
    API_MULTIPART_PARSE_FILE_FIELD(container, "file", file);
    if (file)
    {
        /* 打印文件名称 */
        string filename = partfile->getFilename().getValue("");
        OATPP_LOGD("Multipart", "file='%s'", filename.c_str());
        /* 测试将文件保存到磁盘上面 */
        // string fullPath = "public/static/file/" + filename;
        // file.saveToFile(fullPath.c_str());

        /* 测试上传到FastDFS */
        ZO_CREATE_DFS_CLIENT_URL(dfs, urlPrefix);
        // 获取文件后缀名
        string suffix = "";
        size_t pos = filename.rfind(".");
        if (pos != string::npos)
        {
            suffix = filename.substr(pos + 1);
        }
        // 上传文件
        string downloadUrl = dfs.uploadFile(file->data(), file->size(), suffix);
        downloadUrl = urlPrefix + downloadUrl;
        OATPP_LOGD("Multipart", "download url='%s'", downloadUrl.c_str());
    }

    // 5 响应结果
    /* TODO: 具体响应什么结果，需要根据你的业务需求来，下面是使用示例而已。 */
    auto jvo = StringJsonVO::createShared();
    jvo->success("OK");
    return jvo;
}
```

具体应用的时候, 可能还需要访问mysql获取到对应的图片的路径再去访问文件服务器获取对应的资源(总之就是可能会有其他的交互)


# 二、报表

示例: `zero-one-08mes\mes-cpp\arch-tests\tests\excel\test-excel.cpp`

## 2.1 大自然的馈赠: ExcelComponent.h
- `\zero-one-08mes\mes-cpp\lib-common\include\ExcelComponent.h`
```C++
/**
 * Excel组件
 * 注意：xlnt只支持xlsx文件格式
 */
class ExcelComponent
{
private:
    xlnt::workbook wb;
    xlnt::worksheet sheet;
    // 行高
    double rowHeight = 20;
    // 列宽
    double colWidth = 20;
    // 创建Sheet
    void createSheet(const std::string& sheetName);
public:
    ExcelComponent();
    // 设置行高
    void setRowHeight(double rowHeight);
    // 设置列宽
    void setColWidth(double colWidth);
    // 清空工作薄
    void clearWorkbook();
    
    //************************************
    // Method:    readIntoVector
    // FullName:  ExcelComponent::readIntoVector
    // Access:    public 
    // Returns:   std::vector<std::vector<std::string>> 指定页签内容的二维vector
    // Description: 读取指定文件指定页签的内容
    // Parameter: const std::string & fileName 文件名称的全路径，注意文件路径分隔符使用/
    // Parameter: const std::string & sheetName 页签名称
    //************************************
    std::vector<std::vector<std::string>> readIntoVector(const std::string& fileName, const std::string& sheetName);
    
    //************************************
    // Method:    writeVectorToFile
    // FullName:  ExcelComponent::writeVectorToFile
    // Access:    public 
    // Returns:   void
    // Description: 新增内容到指定页签，并保存到文件中
    // Parameter: const std::string & fileName 文件名称的全路径，注意文件路径分隔符使用/
    // Parameter: const std::string & sheetName 新增内容保存到的页签名称
    // Parameter: const std::vector<std::vector<std::string>> & data 新增保存的数据
    //************************************
    void writeVectorToFile(const std::string& fileName, const std::string& sheetName, const std::vector<std::vector<std::string>>& data);

    // 加载指定Excel文件到内存
    void loadFile(const std::string& filename);
};
```

## 2.2 include

```C++
// xlnt使用需要的相关头文件
#include "Macros.h"         // 获取 yaml 文件配置
#include "ExcelComponent.h" // xlxs
```

## 2.3 ご注意: UTF-8でC++に呪い

```C++
/**
 * #TIP 注意事项：
 * Xlnt中的字符编码必须是utf-8所以不能再代码中出现中文字硬编码的形式;
 * 如果需要写中文，通过yaml配置文件的方式书写中文
 */

// 定义保存数据位置和页签名称
// 注意：文件件名称和文件路径不能出现中文
std::string fileName = "./public/excel/1.xlsx";
// 注意：因为xlnt不能存储非utf8编码的字符，所以中文字需要从配置文件中获取
std::string sheetName = ZH_WORDS_GETTER("excel.sheet.s1");
```

许多东西都需要从配置文件获取!

## 2.4 小测试
- 写了点小练习, 注意不要出现中文在代码的字符串中! (还有读取/覆盖等没有演示到, 另外样式则需要去官方文档学习: [`xlnt`excel报表](https://github.com/tfussell/xlnt))

```C++
auto testFun = []() -> int { // 测试学习使用
    // 创建测试数据
    std::vector<std::vector<std::string>> data(3, std::vector<std::string>(5));
    {
        int cnt = 1;
        for (auto& it : data) {
            it[0] = to_string(cnt) + " big xianMu";
            it[1] = to_string(cnt) + " tuble man";
            it[2] = to_string(cnt) + " pasen";
            ++cnt;
        }
    }
    
    // 插入表头
    data.insert(data.begin(), {
        ZH_WORDS_GETTER("hx.tabs.tab_1"),
        ZH_WORDS_GETTER("hx.tabs.tab_2"),
        ZH_WORDS_GETTER("hx.tabs.tab_3"),
    });

    std::string fileName = "./public/excel/heng_xin.xlsx";   // 路径
    std::string sheetName = ZH_WORDS_GETTER("hx.file_name"); // 表名
    // 保存到文件
    ExcelComponent excel;
    excel.writeVectorToFile(fileName, sheetName, data);

    return 0;
}();
```

# 三、声明式服务
## 3.1 什么是声明式服务
简单地说就是 `在我们代码中, 只有声明没有实现` 的服务. 说白了就是 A 服务调用 B 服务 (通过http(套接字通信))

## 3.2 如何使用
### 3.2.1 注入组件

`main.cpp`的`HttpServer::startServer`的`otherComponentRegCall`参数, 允许外部注册组件通过该回调执行组件注册.
```C++
// 启动HTTP服务器
HttpServer::startServer(ServerInfo::getInstance().getServerPort(),
    [=](Endpoints* doc, HttpRouter* router) {
        Router(doc, router).initRouter();
    },
    [](std::shared_ptr<AbstractComponentReg>* o) {
        *o = std::make_shared<OtherComponent>();
    }
);
```

具体会在`HttpServer::init`中回调.

```C++
// 注册其它组件
std::shared_ptr<AbstractComponentReg> other = nullptr;
if (otherComponentRegCall) {
    otherComponentRegCall(&other);
}
```

而`AbstractComponentReg`是一个抽象类, 使用的话要在子类写(`mes-cpp\arch-demo\controller\OtherComponent.hpp`)

```C++
/**
 * 其它Oatpp组件注册附件，后续如果需要附加其它组件可以在这里进行外部定义
 */
class OtherComponent : public AbstractComponentReg
{
#ifdef HTTP_SERVER_DEMO
    // 定义一个WebSocket组件用于演示WebSocket的使用
    OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::network::ConnectionHandler>, websocketConnectionHandler)("websocket", [] {
        auto connectionHandler = oatpp::websocket::ConnectionHandler::createShared();
        connectionHandler->setSocketInstanceListener(std::make_shared<WSInstanceListener>());
        return connectionHandler;
        }());
    // 定义一个示例RequestExecutor组件用于发送api请求
    OATPP_CREATE_COMPONENT(std::shared_ptr<oatpp::web::client::HttpRequestExecutor>, sampleApiExecutor)("sample-api", [] {
        auto connectionProvider = oatpp::network::tcp::client::ConnectionProvider::createShared({ "192.168.213.88", 10100 });
        return oatpp::web::client::HttpRequestExecutor::createShared(connectionProvider);
        }());
#endif
    // #TIP: 项目中需要注册其他组件在下面书写组件注册代码

};
```

### 3.2.2 编写 控制层

这个控制层依旧是我们的后端的(和之前写的大差不差)

```C++
class SampleController : public oatpp::web::server::api::ApiController // 1 继承控制器
{
    // 2 定义控制器访问入口
    API_ACCESS_DECLARE(SampleController);
    // 3 定义接口
public:
    // 3.1 定义测试声明式服务调用的接口1描述
    ENDPOINT_INFO(queryOne) {
        // 定义标题和返回类型以及授权支持
        API_DEF_ADD_COMMON_AUTH(ZH_WORDS_GETTER("sample.query-one.summary"), SampleJsonVO::Wrapper);
        // 定义其他路径参数说明
        API_DEF_ADD_QUERY_PARAMS(UInt64, "id", ZH_WORDS_GETTER("sample.field.id"), 1, true);
    }
    // 3.2 定义测试声明式服务调用的接口1处理
    API_HANDLER_ENDPOINT_AUTH(API_M_GET, "/sample/query-one", queryOne, QUERY(UInt64, id), execQueryOne(id, authObject->getPayload()));

    // 3.1 定义测试声明式服务调用的接口2描述
    ENDPOINT_INFO(queryAll) {
        // 定义标题和返回类型以及授权支持
        API_DEF_ADD_COMMON_AUTH(ZH_WORDS_GETTER("sample.query-all.summary"), SamplePageJsonVO::Wrapper);
        // 定义分页查询参数描述
        API_DEF_ADD_PAGE_PARAMS();
        // 定义其他查询参数描述
        API_DEF_ADD_QUERY_PARAMS(String, "name", ZH_WORDS_GETTER("sample.field.name"), "li ming", false);
    }
    // 3.2 定义测试声明式服务调用的接口1处理
    API_HANDLER_ENDPOINT_QUERY_AUTH(API_M_GET, "/sample/query-all", queryAll, SampleQuery, execQueryAll(query, authObject->getPayload()));
private:
    // 3.3 测试声明式服务调用1
    SampleJsonVO::Wrapper execQueryOne(const UInt64& id, const PayloadDTO& payload);
    // 3.3 测试声明式服务调用2
    SamplePageJsonVO::Wrapper execQueryAll(const SampleQuery::Wrapper& query, const PayloadDTO& payload);
};

SampleJsonVO::Wrapper SampleController::execQueryOne(const UInt64& id, const PayloadDTO& payload)
{
    // 创建客户端对象
    API_CLIENT_CREATE(ac, om, SampleApiClient, "sample-api");
    // 构建凭证
    std::string token = PayloadDTO::getTokenPrefix() + payload.getToken();
    // 返回查询结果
    return ac->queryById(token, id)->readBodyToDto<SampleJsonVO::Wrapper>(om);
}

SamplePageJsonVO::Wrapper SampleController::execQueryAll(const SampleQuery::Wrapper& query, const PayloadDTO& payload)
{
    // 创建客户端对象
    API_CLIENT_CREATE(ac, om, SampleApiClient, "sample-api");
    // 构建凭证
    std::string token = PayloadDTO::getTokenPrefix() + payload.getToken();
    // 返回查询结果
    return ac->queryAll(token, query->pageIndex, query->pageSize, URIUtil::urlEncode(query->name))->readBodyToDto<SamplePageJsonVO::Wrapper>(om);
}
```

此处的`API_CLIENT_CREATE`就是调用服务层(见下)

### 3.2.3 编写 服务层 (重中之重)

```C++
// 1 导入必须头文件
#include "oatpp/web/client/ApiClient.hpp"
#include "ApiHelper.h"

/**
 * 书写示例ApiClient来演示定义声明式服务
 */
class SampleApiClient : public oatpp::web::client::ApiClient // 2 继承这个类
{
    // 3 定义ApiClient控制器使用宏---开始
#include OATPP_CODEGEN_BEGIN(ApiClient)
    
    // 4 初始化定义
    API_CLIENT_INIT(SampleApiClient);
    
    // 5 使用API_CALL或API_CALL_ASYNC声明调用服务接口

    // 通过ID查询
    API_CALL(API_M_GET, "/sample/query-one", queryById, API_HANDLER_AUTN_HEADER, QUERY(UInt64, id));
    // 多条件查询
    API_CALL(API_M_GET, "/sample/query-all", queryAll, API_HANDLER_AUTN_HEADER, API_HANDLER_PAGE_PARAME, QUERY(String, name));
    
#include OATPP_CODEGEN_END(ApiClient)
    // 3 取消ApiClient控制器使用宏---结束
};
```

然后就不要理会DAO层了, 因为交给外部服务做了, 然后就等外部服务返回给后端(服务层), 服务层再返回给控制层, 控制层返回给前端就OK.