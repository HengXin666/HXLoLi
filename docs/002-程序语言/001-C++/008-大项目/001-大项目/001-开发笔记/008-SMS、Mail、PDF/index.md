# 一、SMS

注册太太太麻烦了, 还需要手持身份证?! ...艹

<p style="text-align:center">!!! 使用只能在Linux环境下 !!!</p>

## 1.1 大自然的馈赠: `AliSmsSender.h`
- `zero-one-08mes\mes-cpp\lib-common\include\sms\aliyun\AliSmsSender.h`

```C++
/**
 * 书写一个短信发送组件
 * 注意：
 * 1、此组件不支持Windows平台，要使用此组件请在Linux环境下面使用
 * 2、使用的时候需要定义AliKeyConfig类相关成员变量和成员函数，用于指定ak和sk相关值
 */
class AliSmsSender
{
public:
    //************************************
    // Method:    AliSmsSender
    // FullName:  AliSmsSender::AliSmsSender
    // Access:    public 
    // Returns:   
    // Description: 构造初始化
    // Parameter: const std::string& keyfilename key配置文件名称
    // Parameter: const std::string& prefix 配置读取前缀
    //************************************
    AliSmsSender(const std::string& keyfilename = "key-config.yaml", const std::string& prefix = "aliyun.sms");
    // 析构
    ~AliSmsSender();
    //************************************
    // Method:    sendSms
    // FullName:  AliSmsSender::sendSms
    // Access:    public 
    // Returns:   std::shared_ptr<AliSmsResult> 发送结果
    // Description: 发送单条信息
    // Parameter: const std::string& phoneNumber 手机号码
    // Parameter: const std::string& signName 签名名称。如：阿里云短信测试
    // Parameter: const std::string& templateCode 短信模板CODE。如：SMS_15******
    // Parameter: const std::string& templateParams 模板参数值，json数据格式。如：{"code":"996007"}
    //************************************
    std::shared_ptr<AliSmsResult> sendSms(
        const std::string& phoneNumber,
        const std::string& signName,
        const std::string& templateCode,
        const std::string& templateParams);
    //************************************
    // Method:    sendSmsBatch
    // FullName:  AliSmsSender::sendSmsBatch
    // Access:    public 
    // Returns:   std::shared_ptr<AliSmsResult>
    // Description: 批量发送短信，该接口单次最多支持100个号码。注意批量发送所有参数格式都是json格式字符串
    // Parameter: const std::string & phoneNumbers 手机号码列表。如：["132xxxxxxx","159xxxxxxx"]
    // Parameter: const std::string & signNames 短信签名名称，短信签名的个数必须与手机号码的个数相同、内容一一对应。
    //                                                 如：["签名1","签名2"]
    // Parameter: const std::string & templateCode 短信模板CODE。如：SMS_15******
    // Parameter: const std::string & templateParams 短信模板变量对应的实际值，模板变量值的个数必须与手机号码、签名的个数相同、内容一一对应，表示向指定手机号码中发对应签名的短信，且短信模板中的变量参数替换为对应的值。
    //                                                 如：[{"code":"996007"},{"code":"857857"}]
    //************************************
    std::shared_ptr<AliSmsResult> sendSmsBatch(
        const std::string& phoneNumbers,
        const std::string& signNames,
        const std::string& templateCode,
        const std::string& templateParams);
};
```

## 1.2 示例
### 1.2.1 单条发送

```C++
// 测试单条发送
TEST(AliSmsTest, SendOne) {
    AliSmsSender sender;
    // 注意：因为参数中有中文字符，所以所有中文字从中文词典中获取，不然会发送短信失败
    std::shared_ptr<AliSmsResult> res = sender.sendSms("电话号码", "阿里云短信测试", "SMS_154950909", "{\"code\":\"996007\"}");
    ASSERT_EQ(res->Code(), "OK");
    std::cout
        << "code:  " << res->Code() << std::endl
        << "msg:   " << res->Message() << std::endl
        << "bizid: " << res->BizId() << std::endl
        << "reqid: " << res->RequestId() << std::endl;
}
```

### 1.2.2 多条发送

可能有失败的风险(免费的可能不支持哦~)

```C++
// 测试多条发送
TEST(AliSmsTest, SendBatch)
{
    AliSmsSender sender;
    // 注意：因为参数中有中文字符，所以所有中文字从中文词典中获取，不然会发送短信失败
    std::shared_ptr<AliSmsResult> res = sender.sendSmsBatch("[\"号码1\",\"号码2\"]", "[\"签名1\",\"签名2\"]", "SMS_********", "[{\"code\":\"996007\"},{\"code\":\"857857\"}]");
    ASSERT_EQ(res->Code(), "OK");
    std::cout
        << "code:  " << res->Code() << std::endl
        << "msg:   " << res->Message() << std::endl
        << "bizid: " << res->BizId() << std::endl
        << "reqid: " << res->RequestId() << std::endl;
}
```
调用API你应该会的吧よね

# 二、Mail

## 2.1 大自然的馈赠: `EmailSender.h`
- `zero-one-08mes\mes-cpp\lib-common\include\EmailSender.h`

邮件发送工具类
```C++
/**
 * 书写一个邮件发送工具类
 */
class EmailSender
{
public:
    //************************************
    // Method:    EmailSender
    // FullName:  EmailSender::EmailSender
    // Access:    public 
    // Returns:   
    // Description: 构造初始化
    // Parameter: const std::string& smtp_server 邮件服务器，如smtp.163.com
    // Parameter: const int smtp_port 服务器端口，如25
    // Parameter: const std::string& password 授权密码
    // Parameter: const std::string& from_email 邮件发送人邮箱地址
    // Parameter: const std::string& from_name  邮件发送人名称，默认值No-Reply
    // Parameter: const std::string& charset 内容编码，默认值gb2312
    //************************************
    EmailSender(
        const std::string& smtp_server,
        const int smtp_port,
        const std::string& password,
        const std::string& from_email,
        const std::string& from_name = "No-Reply",
        const std::string& charset = "gb2312");
    ~EmailSender();
    // 设置邮件主题和内容，可以是HTML格式或纯文本
    void setEmailContent(const std::string& subject = "", const std::string& body = "");
    // 添加邮件接收人
    void addRecvEmailAddr(const std::string& email_addr, const std::string& name = "");
    // 添加邮件抄送人
    void addCcEmailAddr(const std::string& email_addr, const std::string& name = "");
    // 添加附件
    void addAttachment(const std::string& filename);
    // 执行发送
    bool send();
    // 获取编码
    std::string getCharset() const;
    // 设置编码
    void setCharset(std::string val);
    // 启用SSL协议
    void enableSSL();
    // 禁用SSL协议
    void disableSSL();
private:
    // smtp服务器
    std::string m_smtp_url;
    // 内容编码，默认gb2312
    std::string m_charset;
    // 邮件发送人 key 邮件地址 val 发送人名称
    std::pair<std::string, std::string> m_from;
    // 邮件服务器授权密码
    std::string m_password;
    // 邮件接收人 key 邮件地址 val 接收人名称
    std::vector<std::pair<std::string, std::string>> m_recvs;
    // 邮件抄送人 key 邮件地址 val 抄送人名称
    std::vector<std::pair<std::string, std::string>> m_ccs;
    // 邮件主题
    std::string m_email_subject;
    // 邮件内容
    std::string m_email_body;
    // 邮件附件文件列表（文件的绝对或相对路径）
    std::vector<std::string> m_attachments;
    // 回调函数，将MIME协议的拼接的字符串由libcurl发出
    static size_t payloadSource(void* ptr, size_t size, size_t nmemb, void* stream);
    // 创建邮件MIME内容
    std::string generateMimeMessage();
    // 获取附件文件名
    void getFileName(const std::string& path, std::string& filename);
    // 获取附件文件类型  
    void getFileContentType(const std::string& path, std::string& contentType);
};
```

<!-- DEUQILIYRORCJFUZ -->

## 2.2 配置好你的 Nacos !

```C++
/** 定义一个Nacos客户端对象，用于获取配置 ip:端口, 命名空间 (必须要开一个命名空间!)*/
NacosClient _ns("192.168.213.88:8848", "ed997686-a122-40ca-9b3c-15dda460fcaf");
```
## 2.3 示例
### 2.3.1 创建客户端管理类
```C++
/**
 * 定义一个测试夹具类，用于管理邮件发送对象创建
 */
class MailTest : public testing::Test {
protected:
    void SetUp() override
    {
        /** 定义一个Nacos客户端对象，用于获取配置 */
        NacosClient _ns("192.168.213.88:8848", "ed997686-a122-40ca-9b3c-15dda460fcaf");
        /** 读取配置数据节点 */
        auto _keyConfig = _ns.getConfig("key-config.yaml");
        /** 定义客户端对象 */
        YamlHelper _yh;
        this->emailSender = new EmailSender(
            _yh.getString(&_keyConfig, "spring.mail.host"),
            std::stoi(_yh.getString(&_keyConfig, "spring.mail.port")),
            _yh.getString(&_keyConfig, "spring.mail.password"),
            _yh.getString(&_keyConfig, "spring.mail.username"),
            _yh.getString(&_keyConfig, "spring.mail.properties.name"));
        this->emailSender->setCharset("utf8");
    }
    void TearDown() override
    {
        if (emailSender)
        {
            delete emailSender;
            emailSender = nullptr;
        }
    }
    // 邮件发送客户端对象
    EmailSender* emailSender = nullptr;
    // 测试邮件主题
    std::string topic = ZH_WORDS_GETTER("mail.topic");
};
```

### 2.3.2 发送文本内容

```C++
// 测试发送文本内容
TEST_F(MailTest, SendText) {
    std::string body = ZH_WORDS_GETTER("mail.body2"); // 发送的正文
    emailSender->addRecvEmailAddr("282000500@qq.com", "Heng_Xin"); // 收件人邮箱, 收件人名称
    emailSender->setEmailContent(topic, body);
    emailSender->addCcEmailAddr("282000500@qq.com", "loli"); // 添加抄送人
    ASSERT_EQ(emailSender->send(), true);
}
```

### 2.3.3 发送Html内容

```C++
// 测试发送Html内容
TEST_F(MailTest, SendHtml) {
    std::string body = ZH_WORDS_GETTER("mail.body1");
    emailSender->addRecvEmailAddr("282000500@qq.com", "Heng_Xin");
    emailSender->setEmailContent(topic, body);
    ASSERT_EQ(emailSender->send(), true);
}
```

### 2.3.4 发送附件

```C++
// 测试发送附件
TEST_F(MailTest, SendAttach) {
    std::string body = ZH_WORDS_GETTER("mail.body1");
    emailSender->addRecvEmailAddr("282000500@qq.com", "Heng_Xin");
    emailSender->setEmailContent(topic, body);
    emailSender->addAttachment("/root/dockerApp/dfsFile/Dockerfile");
    ASSERT_EQ(emailSender->send(), true);
}
```


# 三、PDF & 二维码 / 条形码
## 3.1 大自然的馈赠: `PdfComponent.h`

```C++
class PdfComponent;
// 定义一个模板渲染函数
using PdfTplRenderCf = void(*)(YAML::Node*, PdfComponent*, void* realData);

/**
 * 书写一个PDF组件，用于封装libharu库常用操作
 * 注意：libharu坐标系是原点在左下角不是左上角
 */
class PdfComponent final
{
private:
    // 文档句柄
    HPDF_Doc _doc;
    // 当前页面句柄
    HPDF_Page _currPage;
    // 记录注册的渲染模板
    static std::map<std::string, PdfTplRenderCf> tplRender;
public:
    // 构造初始化
    PdfComponent();
    // 析构释放资源
    ~PdfComponent();
    // 获取文档句柄
    HPDF_Doc getDoc() const;
    // 获取当前页面句柄
    HPDF_Page getCurrPage() const;
    //************************************
    // Method:    getBase14Font
    // FullName:  PdfComponent::getBase14Font
    // Access:    public 
    // Returns:   HPDF_Font
    // Description: 获取PDF内置PBase14字体句柄，更多参考：https://github.com/libharu/libharu/wiki/Fonts
    // Parameter: const std::string& name 字体名称
    //************************************
    HPDF_Font getBase14Font(const std::string& name);
    //************************************
    // Method:    getCnSFont
    // FullName:  PdfComponent::getCnSFont
    // Access:    public 
    // Returns:   HPDF_Font
    // Description: 获取Haru内置中文简体字体句柄
    // Parameter: const std::string& name 字体名称，目前支持：SimSun, SimHei
    // Parameter: bool isVertical 是否纵向显示
    //************************************
    HPDF_Font getCnSFont(const std::string& name, bool isVertical = false);
    //************************************
    // Method:    getCnTFont
    // FullName:  PdfComponent::getCnTFont
    // Access:    public 
    // Returns:   HPDF_Font
    // Description: 获取Haru内置中文繁体字体句柄，目前只支持一种：MingLiU
    // Parameter: bool isVertical 是否纵向显示
    //************************************
    HPDF_Font getCnTFont(bool isVertical = false);
    //************************************
    // Method:    getTtFont
    // FullName:  PdfComponent::getTtFont
    // Access:    public 
    // Returns:   HPDF_Font
    // Description: 获取TTF字体
    // Parameter: const std::string& fontPath 字体文件全路径
    // Parameter: const std::string& encoding 编码名称，默认为CP1252
    //************************************
    HPDF_Font getTtFont(const std::string& fontPath, const std::string& encoding = "CP1252");
    //************************************
    // Method:    getNewPage
    // FullName:  PdfComponent::getNewPage
    // Access:    public 
    // Returns:   HPDF_Page
    // Description: 获取一个新的页面
    // Parameter: HPDF_PageSizes pageSize 页面大小，默认A4
    // Parameter: HPDF_PageDirection direction 页面方向，默认纵向
    //************************************
    HPDF_Page getNewPage(HPDF_PageSizes pageSize = HPDF_PageSizes::HPDF_PAGE_SIZE_A4, HPDF_PageDirection direction = HPDF_PageDirection::HPDF_PAGE_PORTRAIT);
    //************************************
    // Method:    drawText
    // FullName:  PdfComponent::drawText
    // Access:    public 
    // Returns:   void
    // Description: 绘制文字
    // Parameter: const std::string& text 文字内容
    // Parameter: HPDF_REAL posx x坐标
    // Parameter: HPDF_REAL posy y坐标
    // Parameter: HPDF_Page page 页面句柄，默认值为当前页面句柄
    //************************************
    void drawText(const std::string& text, HPDF_REAL posx, HPDF_REAL posy, HPDF_Page page = 0);
    //************************************
    // Method:    drawTextCenter
    // FullName:  PdfComponent::drawTextCenter
    // Access:    public 
    // Returns:   void
    // Description: 绘制文字，文字在页面中居中显示
    // Parameter: const std::string& text 文字内容
    // Parameter: HPDF_Page page 页面句柄，默认值为当前页面句柄
    //************************************
    void drawTextCenter(const std::string& text, HPDF_Page page = 0);
    //************************************
    // Method:    drawTextCenterH
    // FullName:  PdfComponent::drawTextCenterH
    // Access:    public 
    // Returns:   void
    // Description: 绘制文字，让文字水平居中显示
    // Parameter: const std::string& text 文字内容
    // Parameter: HPDF_REAL posy y坐标
    // Parameter: HPDF_Page page 页面句柄，默认值为当前页面句柄
    //************************************
    void drawTextCenterH(const std::string& text, HPDF_REAL posy, HPDF_Page page = 0);
    //************************************
    // Method:    drawTextCenterV
    // FullName:  PdfComponent::drawTextCenterV
    // Access:    public 
    // Returns:   void
    // Description: 绘制文字，让文字垂直居中显示
    // Parameter: const std::string& text 文字内容
    // Parameter: HPDF_REAL posx x坐标
    // Parameter: HPDF_Page page 页面句柄，默认值为当前页面句柄
    //************************************
    void drawTextCenterV(const std::string& text, HPDF_REAL posx, HPDF_Page page = 0);
    //************************************
    // Method:    registerTplRender
    // FullName:  PdfComponent::registerTplRender
    // Access:    public static 
    // Returns:   void
    // Description: 注册渲染模板
    // Parameter: const std::string& tplName 模板名称
    // Parameter: PdfTplRenderCf cf 渲染回调函数
    //************************************
    static void registerTplRender(const std::string& tplName, PdfTplRenderCf cf);
    //************************************
    // Method:    drawWithTemplate
    // FullName:  PdfComponent::drawWithTemplate
    // Access:    public 
    // Returns:   void
    // Description: 通过模板绘制
    // Parameter: const std::string& tplPath 模板配置文件
    // Parameter: const std::string& tplName 模板名称
    // Parameter: void* realData 实时运行数据，用于传递动态数据
    //************************************
    void drawWithTemplate(const std::string& tplPath, const std::string& tplName, void* realData);
    //************************************
    // Method:    saveDocToFile
    // FullName:  PdfComponent::saveDocToFile
    // Access:    public 
    // Returns:   bool 保存成功返回true
    // Description: 保存文档到文件
    // Parameter: const std::string& fullPath 文件全路径
    //************************************
    bool saveDocToFile(const std::string& fullPath);
};
```

## 3.2 示例
### 3.2.1 渲染文字界面

```C++
TEST(PdfTest, TestText) {
    PdfComponent pdf;
    // 创建一个页面
    HPDF_Page newPage = pdf.getNewPage();
    // 设置页面字体
    HPDF_Page_SetFontAndSize(newPage, pdf.getCnSFont("SimSun"), 20);
    // 绘制字体
    pdf.drawTextCenterH(ZH_WORDS_GETTER("pdf.title"), HPDF_Page_GetHeight(newPage) - 20);
    pdf.drawTextCenter(ZH_WORDS_GETTER("pdf.content"));
    pdf.drawTextCenterH(ZH_WORDS_GETTER("pdf.foot"), 20);
    // 保存到文件
    ASSERT_EQ(pdf.saveDocToFile("test-text.pdf"), true);
}
```

### 3.2.2 渲染条形码界面

```C++
// 实现一个保存条形码的函数
// 条形码在线预览 https://zxing-cpp.github.io/zxing-cpp/demo_writer.html
void addBarCodeToPdf(PdfComponent* pdf) {
    // 设置条码绘制相关参数
    int width = 200, height = 20; // 长宽
    int margin = 10; // 间距
    int eccLevel = 0; // ecc等级0 -10
    CharacterSet encoding = CharacterSet::UTF8; // 编码
    BarcodeFormat format = BarcodeFormatFromString("Code93"); //格式化类型
    // 定义条码中的内容
    std::string input = "11111111";
    // 条码临时保存位置
    std::string filepath = "tmp.jpg";
    try {
        // 生成图形
        auto writer = MultiFormatWriter(format).setMargin(margin).setEncoding(encoding).setEccLevel(eccLevel);
        BitMatrix matrix = writer.encode(input, width, height);
        auto bitmap = ToMatrix<uint8_t>(matrix);
        // 保存到文件
        int success = stbi_write_jpg(filepath.c_str(), bitmap.width(), bitmap.height(), 1, bitmap.data(), 0);
        // 绘制到PDF
        if (success) {
            // 加载图片文件
            HPDF_Image image = HPDF_LoadJpegImageFromFile(pdf->getDoc(), filepath.c_str());
            // 绘制图片到PDF中
            HPDF_UINT iw = HPDF_Image_GetWidth(image);
            HPDF_UINT ih = HPDF_Image_GetHeight(image);
            HPDF_Page_DrawImage(pdf->getCurrPage(), image, 30, 70, iw, ih);
        }
    } catch (const std::exception& e) {
        std::cerr << e.what() << std::endl;
    }
}

TEST(PdfTest, TestTpl) {
    // 测试注册渲染模板
    PdfComponent::registerTplRender("test", [](YAML::Node* node, PdfComponent* pdf, void* realData) {
        // 创建一个页面
        HPDF_Page newPage = pdf->getNewPage();
        // 设置页面字体
        HPDF_Page_SetFontAndSize(newPage, pdf->getCnSFont("SimSun"), 20);
        // 绘制静态数据
        auto title = (*node)["t"].as<std::string>();
        auto foot = (*node)["f"].as<std::string>();
        pdf->drawTextCenterH(title, HPDF_Page_GetHeight(newPage) - 20);
        pdf->drawTextCenterH(foot, 20);
        // 绘制动态数据
        std::string content = static_cast<std::string*>(realData)->c_str();
        pdf->drawTextCenter(content);
        // 绘制一个条码
        addBarCodeToPdf(pdf);
        // 保存到文件
        pdf->saveDocToFile("test-tpl.pdf");
    });

    // 测试绘制
    PdfComponent pdf;
    auto content = ZH_WORDS_GETTER("pdf.content");
    pdf.drawWithTemplate("tpl/test.yaml", "test", &content);
}
```

本质上是保存一个条形码图片, 再把这个图片读取并写到pdf中, 故同理你也可以绘制图片了

剩下的就是API绘制那些了, 没有什么好教的了..