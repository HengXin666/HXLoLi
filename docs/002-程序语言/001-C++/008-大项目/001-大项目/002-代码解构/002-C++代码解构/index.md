# C++大项目代码解构
## 0. 快速测试 & 环境细节

万用凭证: (和时间有关, 需要定期更新)
```C++
eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJhdXRob3JpdGllcyI6WyJTVVBFUl9BRE1JTiJdLCJleHAiOjE3MjAxODc4ODYsImlkIjoiMSIsInVzZXJfbmFtZSI6InJvdW1pb3UifQ.Xy7Ayv8-5CyqkW9MFITzJZnGxL7NAIRMDMV-Yc7qbOBwaCEci3hoE8SFTeRnrUcZ911dfRuq5pnFp6muweLsAW6xbYruO-ryDq7m_oXoKFWlwH6uaFP-l4yCLJ7xoNkaoBhw_b7fpf5vMngWa9rdNmBiLZu9E2gjyz2Y2MEN_Q90LPHBI7QGndMs-v8YoadymY4AKTmQBDcQPmzo4uu3lCMTLzHyMgYbWlqs4upcgjcb-VVQ-SdT3-vE1SRiotW2s9IEAsermEa9RGtr8Ufd_zNSYww9tA149_UMO8DXXcNQZROy9U0aVRZ7q7rvIROYQH8cbXTaqbDCjvQBZi4bWg
```

快速测试:

data-source.yaml:
```yaml
spring:
  datasource:
    url: jdbc:mysql://192.168.213.88:3306/test?useUnicode=true&useSSL=false&characterEncoding=utf-8&serverTimezone=Asia/Shanghai&allowPublicKeyRetrieval=true
    username: root
    password: 123456
  redis:
    host: 192.168.213.88
    port: 6379
    password: 01star
  data:
    mongodb:
      uri: mongodb://awei:123456@192.168.213.88:27017/firstDb
```

third-services.yaml:
```yaml
rocket-mq:
  name-server: 192.168.213.88:9876
fastdfs:
  charset: UTF-8
  connect-timeout: 5
  network-timeout: 30
  http-secret-key: FastDFS1234567890
  http-anti-steal-token: true
  connection-pool-max-idle: 20
  connection-pool-max-total: 20
  connection-pool-min-idle: 2
  nginx-servers: 192.168.213.88:8888
  tracker-servers: 192.168.213.88:22122
```


## 1. 文件分支

~~(我是八嘎, 全部项目代码都看完, 笔记记录完了才发现, 项目总结有每一个小组的负责功能看qwq(虽然是`mes-c%d-%d`格式))~~

```C++
mes-cpp
├─mes-c1-equipmentmanagement         // 设备管理-类型设置
├─mes-c1-workfixture                 // 工装夹具
├─mes-c1-workstation                 // 基础数据-工作站
├─mes-c2-equipmentledger             // 设备管理-设备台账模块
├─mes-c2-inspection.maintenanceitems // 设备管理-设备台账模块
├─mes-c2-inspection.maintenanceplan  // 设备管理-点检/保养计划模块
├─mes-c2-repairorder                 // 设备管理-维修单模块/维修内容
├─mes-c3-promanager                  // 生产管理-工艺流程/工序设置
├─mes-c3-quamanager                  // 质量管理-常见缺陷
├─mes-c4-prodmgmt                    // 生产管理-生产工单/生产排产/生产报工
├─mes-c4-whmgmt                      // 仓储管理-采购入库
├─mes-c5-QualityControl              // 质量管理-过程检验/检测模板/检测项设置/待检任务
├─mes-c5-WarehouseManagement         // 仓储管理-库存现有量
├─mes-c6-quality                     // 质量管理-来料检验
├─mes-c6-storage-product             // 仓储管理-采购入库
├─mes-c6-storage-return              // 仓储管理-采购退货
├─mes-c7-rejection                   // 仓储管理-生产退料
├─mes-c7-settings                    // 仓储管理-仓库设置
├─mes-c7-storage                     // 仓储管理-产品入库
```

## 2. 重要功能总结

- 下列只列出主要功能, 某些功能的特别之处或者问题, 其他默认CRUD:

### 2.1 [问题]: 消失的`mes-c%d-xxx/README.md`

0. 怎么几乎都不写`mes-c%d-xxx/README.md`?
    - 目前就只看到`c2`和`c7`写了 (c1 ~ c7)

### 2.2 [问题]: `FastDFS`在项目中的使用存疑问

1. 对于`FastDFS`在项目中的使用存疑问/问题:
    - Excel是保存在服务器A的, 并且名称都是一样的, 虽然上传到`FastDFS`后名称会变为`FID`; 但是本地文件名称一样会不会存在被覆盖(还在上传, 又有新的保存 (虽然不太可能, ~~因为业务函数是原子的..~~(必需等待文件服务器响应直到超时 | 但是万一可以同时调用多次这个业务函数呢?!)))/混淆的风险?

        - 解决: 使用时间戳+随机数混淆/生成唯一key来确保本地文件id唯一, 防止被覆盖, 并且上传文件服务器后, 删除本地的文件, 防止空间浪费~~(最好是只在内存上, 不需要磁盘io参与, 但是Excel和FastDFS的API好像不支持)~~.
    
    - 以及下载有文件名称乱码: [FastDFS使用之文件名称](https://blog.csdn.net/gobravery/article/details/79353515)


```sh
# nginx配置
location /group1/M00/ {
    root /fastdfs/data; # 注释: 填写数据存储路径，前面root别忘记
    if ($arg_attname ~ "^(.+)") {
        # 设置下载
        add_header Content-Type application/x-download;
        # 设置文件名
        add_header Content-Disposition "attachment;filename=$arg_attname";
    }
    ngx_fastdfs_module;
}
```

重新加载配置文件:
```sh
nginx -s reload
```

  - 另外上传到`FastDFS`应该有留存周期才对, 即[定时自动删除fdfs文件(清理FdastDfs空间)](https://blog.csdn.net/u010976445/article/details/124475846)
  
- 然后, 项目中不止一处把保存文件、上传到DFS（分布式文件系统）以及生成下载链接这些操作放在控制层(Controller), 这样是不合适的, 控制层主要负责接收请求、处理输入输出，以及控制业务流程的顺序，而具体的文件操作和上传逻辑应该在服务层进行。(违反了[[SOLID]单一职责原则](../../../../../../001-计佬常識/002-设计模式/005-面向对象设计原则/002-设计原则/002-【SOLID】单一职责原则/index.md), 还提高了代码的耦合度)

#### 2.2.1 返回链接 (也可以返回`StringJsonVO`)

```C++
// 声明控制层端点
// 导出维修单分页查询 接口描述
ENDPOINT_INFO(queryRepairorderToFile) {
    // 定义接口标题
    API_DEF_ADD_TITLE("[Find Data Return To File]" + ZH_WORDS_GETTER("repairorder.summary.page_query_table"));
    // 定义默认授权参数（可选定义，如果定义了，下面ENDPOINT里面需要加入API_HANDLER_AUTH_PARAME）
    API_DEF_ADD_AUTH();
    // 定义响应参数格式
    //API_DEF_ADD_RSP_JSON_WRAPPER(RepairorderPageJsonVO);
    // 定义分页查询参数描述
    API_DEF_ADD_PAGE_PARAMS();
    // 定义其他查询参数描述
    API_DEF_ADD_QUERY_PARAMS(String, "repairCode", ZH_WORDS_GETTER("repairorder.query.fields.repair_code"), "REP20240721", false);      // 维修单编号
    API_DEF_ADD_QUERY_PARAMS(String, "repairName", ZH_WORDS_GETTER("repairorder.query.fields.repair_name"), "BUG-Table", false);        // 维修单名称
    API_DEF_ADD_QUERY_PARAMS(String, "machineryCode", ZH_WORDS_GETTER("repairorder.query.fields.machinery_code"), "M0001", false);      // 设备编码
    API_DEF_ADD_QUERY_PARAMS(String, "machineryName", ZH_WORDS_GETTER("repairorder.query.fields.machinery_name"), "WG-A1-MK2", false);  // 设备名称
    API_DEF_ADD_QUERY_PARAMS(String, "repairResult", ZH_WORDS_GETTER("repairorder.query.fields.repair_result"), "SUCCESS", false);      // 维修结果 (SUCCESS: 维修成功; SCRAP: 报废)
    API_DEF_ADD_QUERY_PARAMS(String, "status", ZH_WORDS_GETTER("repairorder.query.fields.status"), "PREPARE", false);                   // 单据状态 (PREPARE: 草稿; CONFIRMED: 已确认; APPROVING: 审批中; APPROVED: 已审批; FINISHED: 已完成)
}
// 导出维修单分页查询 接口处理
ENDPOINT(API_M_GET, "/equipment-ledger/repairorder/page-query-table-to-file", queryRepairorderToFile, QUERIES(QueryParams, queryParams), API_HANDLER_AUTH_PARAME) {
    // 解析查询参数为Query领域模型
    API_HANDLER_QUERY_PARAM(pageQuery, RepairorderQuery, queryParams);
    // 呼叫执行函数响应结果
    return queryRepairorderToFile(pageQuery);
}

// 控制层
std::shared_ptr<oatpp::web::server::api::ApiController::OutgoingResponse> RepairorderController::queryRepairorderToFile(const RepairorderQuery::Wrapper& query)
{
    // 定义一个Service
    RepairorderService service;
    // 查询数据
    auto res = service.listAllToFile(query);
    // 响应结果
    if (res == "")
        return createResponse(Status::CODE_400, "Error: ???");
    return createResponse(Status::CODE_200, res + "?arg_attname=table" /* + ZH_WORDS_GETTER("repairorder.summary.page_query_table") */ + ".xlsx"); // 这个去文件名称乱码
}

// 服务层
std::string RepairorderService::listAllToFile(const RepairorderQuery::Wrapper& query)
{
    // 查询数据总条数
    RepairorderDAO dao;
    uint64_t count = dao.count(query);
    if (count <= 0) {
        return "";
    }

    // 分页查询数据
    list<DvRepairDO> result = dao.selectWithPage(query);

    // 将DO保存到文件
    std::vector<std::vector<std::string>> vvexcel;
    vvexcel.insert(vvexcel.begin(), {
               // 定义其他查询参数描述
               ZH_WORDS_GETTER("repairorder.query.fields.repair_id"),      // 维修单ID
               ZH_WORDS_GETTER("repairorder.query.fields.repair_code"),    // 维修单编号
               ZH_WORDS_GETTER("repairorder.query.fields.repair_name"),    // 维修单名称
               ZH_WORDS_GETTER("repairorder.query.fields.machinery_id"),   // 设备ID
               ZH_WORDS_GETTER("repairorder.query.fields.machinery_code"), // 设备编码
               ZH_WORDS_GETTER("repairorder.query.fields.machinery_name"), // 设备名称
               ZH_WORDS_GETTER("repairorder.query.fields.require_date"),   // 报修日期
               ZH_WORDS_GETTER("repairorder.query.fields.finish_date"),    // 维修完成日期
               ZH_WORDS_GETTER("repairorder.query.fields.confirm_date"),   // 验收日期
               ZH_WORDS_GETTER("repairorder.query.fields.repair_result"),  // 维修结果 (SUCCESS: 维修成功; SCRAP: 报废)
               ZH_WORDS_GETTER("repairorder.query.fields.accepted_by"),    // 维修人员
               ZH_WORDS_GETTER("repairorder.query.fields.confirm_by"),     // 验收人员
               ZH_WORDS_GETTER("repairorder.query.fields.status"),         // 单据状态 (PREPARE: 草稿; CONFIRMED: 已确认; APPROVING: 审批中; APPROVED: 已审批; FINISHED: 已完成)
        });

    for (const DvRepairDO& sub : result) {
        std::vector<string> v;
        v.push_back(to_string(sub.getRepair_id()));
        v.push_back(sub.getRepair_code());
        v.push_back(sub.getRepair_name());
        v.push_back(to_string(sub.getMachinery_id()));
        v.push_back(sub.getMachinery_code());
        v.push_back(sub.getMachinery_name());
        v.push_back(sub.getRequire_date());
        v.push_back(sub.getRequire_date());
        v.push_back(sub.getFinish_date());
        v.push_back(sub.getConfirm_date());
        v.push_back(sub.getRepair_result());
        v.push_back(sub.getAccepted_by());
        v.push_back(sub.getConfirm_by());
        v.push_back(sub.getStatus());
        vvexcel.push_back(v);
    }

    const std::string tempfilePath = "public/execl/repairorder_table_" // 不能用中文!
        + SimpleDateTimeFormat::formatWithMilli("%Y-%m-%d_%H%M%S", "_")
        + ".xlsx";
    // 生成excel文件
    ExcelComponent excel;
    excel.writeVectorToFile(tempfilePath, ZH_WORDS_GETTER("repairorder.summary.page_query_table"), vvexcel);
    
    ZO_CREATE_DFS_CLIENT_URL(dfs, urlPrefix);

    // 上传文件
    string downloadUrl = dfs.uploadFile(tempfilePath);

    OATPP_LOGD("Multipart", "download url='%s'", downloadUrl.c_str()); // 日志

    // 删除本地生成的excel文件
    std::remove(tempfilePath.c_str());

    if (downloadUrl == "")
        return "";

    return urlPrefix + downloadUrl;
}
```

#### 2.2.2 一种使用oatpp而不是DFS的实现

- 相关知识: [oatpp | ResponseFactory.hpp](https://oatpp.io/api/latest/oatpp/web/protocol/http/outgoing/ResponseFactory/)

- 选自`mes-c3-quamanager`(文件处理最好写在服务层, 而非控制层):

| ##container## |
|:--:|
|![Clip_2024-07-03_17-59-48.png ##w800##](./Clip_2024-07-03_17-59-48.png)|

注意是`form-data`格式请求, 具体使用示例在`.\arch-demo\controller\file\FileController.h`有

代码: (看起来, 是没有上传到DFS, 而是直接文件流传输)
```C++
// 导出缺陷
ENDPOINT_INFO(exportDefect) {
    API_DEF_ADD_COMMON_AUTH(ZH_WORDS_GETTER("defect.export.summary"), Void);
}
ENDPOINT(
    API_M_POST,
    "/defect/export-defect",
    exportDefect,
    REQUEST(std::shared_ptr<IncomingRequest>, request),
    API_HANDLER_AUTH_PARAME) {
    return execExportDefect(request);
}

// 5 导出缺陷
std::shared_ptr<oatpp::web::server::api::ApiController::OutgoingResponse> 
DefectController::execExportDefect(std::shared_ptr<oatpp::web::server::api::ApiController::IncomingRequest> request) {
    // 解析请求
    API_MULTIPART_INIT(container, reader);
    API_MULTIPART_CONFIG_MEMO_DEFAULT(reader, -1);
    request->transferBody(&reader);
    API_MULTIPART_PARSE_FORM_FIELD_STR(container, "page_num", page_num);
    API_MULTIPART_PARSE_FORM_FIELD_STR(container, "page_size", page_size);
    API_MULTIPART_PARSE_FORM_FIELD_STR(container, "defect_name", defect_name);
    API_MULTIPART_PARSE_FORM_FIELD_STR(container, "index_type", index_type);
    API_MULTIPART_PARSE_FORM_FIELD_STR(container, "defect_level", defect_level);

    // 判断是否合法
    if (!page_num) {
        return createResponse(Status::CODE_400, "page_num must be exist");
    }
    if (!page_size) {
        return createResponse(Status::CODE_400, "page_size must be exist");
    }

    auto query = QcDefectQuery::createShared();
    query->pageIndex = std::stoi(page_num);
    query->pageSize = std::stoi(page_size);
    if (defect_name) {
        query->defect_name = defect_name.getValue({});
    }
    if (index_type) {
        query->index_type = index_type.getValue({});
    }
    if (defect_level) {
        query->defect_level = defect_level.getValue({});
    }

    DefectService service;
    auto res = service.listAll(query);

    // 构造数据
    std::vector<std::vector<std::string>> data;
    data.push_back({ ZH_WORDS_GETTER("defect.field.defect_code"),
                    ZH_WORDS_GETTER("defect.field.defect_name"),
                    ZH_WORDS_GETTER("defect.field.index_type"),
                    ZH_WORDS_GETTER("defect.field.defect_level") });
    for (const auto& row : *res->rows) {
        data.push_back({
            row->defect_code.getValue("Null"), //
            row->defect_name.getValue("Null"), //
            row->index_type.getValue("Null"),  //
            row->defect_level.getValue("Null") //
        });
    }

    // 构造文件并返回
    const string timestamp { std::to_string(time(NULL)) };
    const string filename { "defect_" + timestamp + ".xlsx" };
    const string filepath { "public/static/qc_defect.xlsx" };
    ExcelComponent excel;
    excel.writeVectorToFile(filepath, "Sheet1", data);
    const auto file_string = String::loadFromFile(filepath.c_str());

    // 返回
    auto response = createResponse(Status::CODE_200, file_string);
    response->putHeader(Header::CONTENT_TYPE, "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet");
    response->putHeader("Content-Disposition", "attachment; filename=" + filename); // 指定了文件名称
    return response;
}
```

### 2.3 [疑问]: 批量删除数据在项目中有多种实现

2. 对于批量删除`MySql中`的数据, 项目中有多种实现: ~~(有单个删除, 但只能删除`草稿`等, 不是批量)~~
    - 使用事务, 如果一个删除失败则回溯
    
    - 不使用事务, 因为数据传输是隐藏字段(非前端直接展示给用户输入), 故忽略合法性判断, 失败则不理会, 返回删除成功的操作数量
    
    - 不使用事务, 先for判断id的合法性, 删除List中不合法的, 然后使用字符串流拼接, 批量删除, 返回影响的行数. (`"DELETE FROM table WHERE id in (?, ?, ... ,?)"`)
    
    - 不使用事务, 一整个for循环: 先判断id合法性, 再删除这一个id, 然后计数一个. 最终返回一个`ListJsonVO`给前端, 每一个元素是对应id的执行结果(成功/失败)

- 在请求上, 项目中多采用的是从`请求体`中读取请求列表, 而不是在URL中, 因为示例项目中也只是`/remove/{id}`这样, 只支持单个, 多个就不知道怎么办了

    - 但是实际上, `示例项目`中是支持URL: `/remove/1,2,3`这样多个id删除, 但是它是java写的, 网络上粗略搜索也只有java的回答qwq... (故导致大家普遍采用在`请求体`中传递json列表)
 
    - 但是! 我发现`mes-c3-quamanager`的大佬有非常好的实现:

| ##container## |
|:--:|
|![Clip_2024-07-03_17-15-03.png ##w800##](./Clip_2024-07-03_17-15-03.png)|

- 实际上就是把原本解析为单个`UInt64`改为解析`String`, 通过分割字符串的`,`来格式化成数字, 本质上还是传递了一个`{id}`, 只不过内容是字符串`"1,2,3"`这样.

```C++
// 4 删除缺陷
API_DEF_ENDPOINT_INFO_AUTH(ZH_WORDS_GETTER("defect.remove.summary"), removeDefect, Uint64JsonVO::Wrapper);
API_HANDLER_ENDPOINT_AUTH(             //
    API_M_DEL,                         //
    "/defect/remove-defect/{id-list}", // 看看这是什么?!
    removeDefect,                      //
    PATH(String, ids, "id-list"),      // WDF? 是string!
    execRemoveDefect(ids)              //
);

// 对应方法, HX注: 如果实际写的话, 最好再写一个工具类, 来 string to list<uint_64>
Uint64JsonVO::Wrapper DefectController::execRemoveDefect(const String& ids) {
    // 1%2C10%2C32%2C4 -> 1,10,32,4
    auto jvo = Uint64JsonVO::createShared();
    std::string s = std::regex_replace(ids.getValue("").c_str(), regex("%2C"), ",");
    // 方便触发判断
    s.push_back(',');
    int id = 0;
    for (int i = 0, n = s.size(); i < n; i++) {
        if (std::isdigit(s[i])) {
            id = id * 10 + s[i] - '0';
        }
        else if (s[i] == ',') {
            if (id <= 0) {
                jvo->init(UInt64(-1), RS_PARAMS_INVALID);
                return jvo;
            }
            id = 0;
        }
        // 非数字
        else {
            jvo->init(UInt64(-1), RS_PARAMS_INVALID);
            return jvo;
        }
    }
    s.pop_back();

    DefectService service;
    if (service.remove(s)) {
        jvo->success(UInt64(200));
    }
    else {
        jvo->fail(UInt64(500));
    }
    return jvo;
}
```

### 2.4 [问题]: CV大师泛滥

3. 纯纯复制示例代码:
    - 你要做CV大师, 没问题, 但是请补上注释! 什么`控制器`/`DAO`/`VO`/`DO`/`DTO`数都数不清, 把鼠标移上去, 想看看是什么东西, 然后它写着`示例damo`代码的注释, 我怎么知道这个是干什么的?! 处理好注释关系, 不要让不重要的注释占领了显示的位置! 如:
      - ![Clip_2024-07-02_16-59-04.png ##w400##](./Clip_2024-07-02_16-59-04.png) 一目瞭然
      - ![Clip_2024-07-02_16-59-32.png ##w400##](./Clip_2024-07-02_16-59-32.png) なんだよ？
      - ![Clip_2024-07-02_17-00-24.png ##w600##](./Clip_2024-07-02_17-00-24.png)
      - 无针对任何人, 只是说明问题. 一起努力吧qwq..

### 2.5 [注意]: 返回的是`ClassJsonVO`, 应该初始化为空

4. 对于返回的是`ClassJsonVO`, 应该初始化为空

例如以下代码, 应该如下所示, 返回无效且为空的值, 否则会返回`RepairorderDetailsDTO`中的默认值(就是展示在`swagger-ui`上的默认值).

```C++
// 维修单详情查询
RepairorderDetailsJsonVO::Wrapper RepairorderController::execQueryDetailsRepairorder(const RepairorderDetailsQuery::Wrapper& id) {
    // 定义返回数据对象
    auto jvo = RepairorderDetailsJsonVO::createShared();
    // 参数校验
    if (id->repairId <= 0) {
        auto tmp = RepairorderDetailsDTO::createShared();
        tmp->repairId = -1;
        tmp->repairCode = "";
        tmp->repairName = "";
        tmp->machineryCode = "";
        tmp->machineryName = "";
        tmp->machineryBrand = "";
        tmp->requireDate = "";
        tmp->remark = "";
        jvo->init(tmp, RS_PARAMS_INVALID); // 上传参数异常
        return jvo;
    }
    // 定义一个Service
    RepairorderService service;
    // 执行数据查询
    auto dto = service.getData(id);
    if (dto->repairId == -1)
        jvo->fail(dto); // 构建失败消息 RS_FAIL
    else
        jvo->success(dto);
    // 响应结果
    return jvo;
}

// Service层 同理
RepairorderDetailsDTO::Wrapper RepairorderService::getData(const RepairorderDetailsQuery::Wrapper& id) {
    // 构建返回对象
    auto pages = RepairorderDetailsDTO::createShared();

    // 查询数据总条数
    RepairorderDAO dao;
    uint64_t count = dao.count(id);
    if (count <= 0) {
        pages->repairId = -1;
        pages->repairCode = "";
        pages->repairName = "";
        pages->machineryCode = "";
        pages->machineryName = "";
        pages->machineryBrand = "";
        pages->requireDate = "";
        pages->remark = "";
        return pages;
    }

    list<DvRepairDO> result = dao.selectById(id->repairId);

    for (const DvRepairDO& sub : result) {
        ZO_STAR_DOMAIN_DO_TO_DTO(pages, sub,
            repairId, Repair_id,
            repairCode, Repair_code,
            repairName, Repair_name,
            machineryCode, Machinery_code,
            machineryName, Machinery_name,
            machineryBrand, Machinery_brand,
            requireDate, Require_date,
            remark, Remark)
    }

    return pages;
}
```

- 有些是Java那边负责的, 而不是java和cpp写同一个功能的不同代码实现!

```C++
mes-cpp
├─mes-c1-equipmentmanagement // 设备管理 | (获取设备类型名称树 <使用了[组合模式]的树形内容的读取>) | (GetDeviceTypeDetailDAO::selectByCondition 中代码冗余了吧, set<uint64_t> allid 和 找出根节点并标记のfor可以不写的, 因为数据库中的根结点的父结点ID就是0了, 即使DO默认为0其查询数据库后一定会被值覆盖的)
├─mes-c1-workfixture // 工装夹具 (传入的id列表到数据库查找到相应数据集并导出为Excel文件 (有改请求头文件名, 但是写在控制层))
├─mes-c1-workstation // 工作站
├─mes-c2-equipmentledger // 设备管理-设备台账模块 (下载导入模板: Excel + FastDFS || 批量删除使用了mysql事务, 一个失败全部回滚)
├─mes-c2-inspection.maintenanceitems // 设备管理-设备台账模块 (有: 及时删除本地生成的excel文件, (但是文件名称依旧还是共用, 有覆盖风险))
├─mes-c2-inspection.maintenanceplan // 设备管理-点检/保养计划模块 (有使用时间作为excel文件名, 但是上传后没有删除本地的 | AddDeviceController的AddDeviceDAO::insert有联表查询逻辑 | win无法编译, 只能Linux)
├─mes-c2-repairorder // 设备管理-维修单模块/维修内容 (删除使用事务 || RepaircontentDAO有联表查询(代码实现))
├─mes-c3-promanager // 生产管理-工艺流程 | 工艺流程 (问题: [工艺导出]的导出表格写到控制层了, 应该放在服务层 (有FastDFS疑问) | win无法编译) <-- 这个接口好多! 24个! CRUD
├─mes-c3-quamanager // 质量管理-常见缺陷 (问题: [导出缺陷]的导出表格写到控制层了, 应该放在服务层 (有FastDFS疑问(只是没有删除本地文件), 但是解决了文件命名乱码问题 & 以时间命名名称)) | [导出缺陷]有解析请求处理 & 使用了{}初始化
├─mes-c4-prodmgmt // 生产管理-生产工单/生产排产/生产报工 (有联表查询的sql语句) (获取产品/物料对应的工艺流程: ProductionProcessDAO::query_by_workordercode, 通过 if (query->itemCode) 来决定是否join, 还是挺复杂的, 获取某物品的制作工艺流程 (mysql表也是, 通过两个表, 然后通过工艺路线id和工序id, 下一工序id, 确定不同的工艺的工序以及顺序!)) | (编码问题, win无法编译(Fxxk VS2022)) | (有甘特图数据) | ProdOrderController 实现为空!
├─mes-c4-whmgmt // 仓储管理-采购入库 (win编译报错) (CRUD | [导出单据]有处理文件名乱码)
├─mes-c5-QualityControl // 质量管理-过程检验/检测模板 (win编译报错) | 使用事务可能存在BUG: 有一条if分支没有终止事务就return了 | 导出表格写到控制层了, 应该放在服务层 (有FastDFS疑问), 但是会删除本地文件
├─mes-c5-WarehouseManagement // 仓储管理-库存现有量 (win编译报错) | (有FastDFS疑问), 但是会删除本地文件
├─mes-c6-quality // 质量管理-来料检验 | (InspectController::execLookTable 空实现) | (CRLD没什么好说的)
├─mes-c6-storage-product // 仓储管理-采购入库 | (PickController::executeExecuteGet, PickController::executeRemoveReturn, PickController::executeDownloadFile 空实现)
├─mes-c6-storage-return // 仓储管理-采购退货 (win 编译不过) | 导出表格写到控制层了, 应该放在服务层 (有FastDFS疑问), 但是会删除本地文件 | DAO层问题: `当前日期/时间`有已经封装的字符串时间工具类, 不用自己写了!
├─mes-c7-rejection // 仓储管理-生产退料 (MaterialInformationController::execRemoveMaterialInformation, RawMaterialOpController::execRemoveRawMaterialOp 并不支持批量删除) | (RawMaterialOpController::execQueryRawMaterialOp 不是要导出成文件吗)
├─mes-c7-settings // 仓储管理-仓库设置 | 使用了[组合模式]的树形内容的读取 (批量删除有问题, URL不应该使用`/{id}`, 虽然是走`BODY_DTO`是正确的, 但是URL这样就不太对了 | 另外[删除仓库]并不支持批量删除)
├─mes-c7-storage // 仓储管理-产品入库 MySql根据某些字段使用模糊查询查询(LIKE CONCAT('%',?,'%')) | (storageController::execRemovestorage 不支持批量删除, 类命名也不规范) | (storageController::execQuerystorage 不应该是导出表格吗)
```