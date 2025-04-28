# .h

```C++
#pragma once
/*
 * Copyright Zero One Star. All rights reserved.
 *
 * @Author: Heng_Xin
 * @Date: ${time}$
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * */
#ifndef _HX_FILE_NAME_H_
#define _HX_FILE_NAME_H_

#endif // !_HX_FILE_NAME_H_
```

# .cpp

```C++
/*
 * Copyright Zero One Star. All rights reserved.
 *
 * @Author: Heng_Xin
 * @Date: ${time}$
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * */
#include "stdafx.h"
```


## 各层命名规范

- `Controller/Service/DAO` 层方法命名规约
    - 获取单个对象的方法用 `query/get/select` 做前缀。
    - 获取多个对象的方法用 `query/list/select` 做前缀。
    - 获取统计值的方法用 `count/count/count` 做前缀。
    - 插入的方法用 `add/save/insert` 做前缀。
    - 删除的方法用 `remove/remove/delete` 做前缀。
    - 修改的方法用 `modify/update/update` 做前缀。
    - `Controller`执行逻辑处理方法使用`exec`作前缀。
        - 如接口端点方法名称为`queryByName`，对应的执行方法名则为`execQueryByName`。
    - `API`请求路径命名，前缀（如：功能模块名称）+ 功能名称（多个单词使用-连接），比如下面的示例：
        - `/sys/query-by-name`、`/user/add-user`、`/user/modify-password`。
- 领域模型命名规约
    - DO：`xxxDO`，`xxx` 即为数据表名。
    - DTO：`xxxDTO`，`xxx` 为业务领域相关的名称。
    - Query：`xxxQuery`，`xxx`为业务领域相关的名称。
    - VO：`xxxVO`，`xxx` 一般为网页名称。
    - `POJO` 是 `DO/DTO/BO/VO/Query` 的统称，禁止命名成 `xxxPOJO`。

## domin-Query (分页)

```C++
#pragma once
/*
 Copyright Zero One Star. All rights reserved.

 @Author: Heng_Xin
 @Date: $(time)

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/
#ifndef _HX_QUERY_
#define _HX_QUERY_

#include "../../GlobalInclude.h"
#include "domain/query/PageQuery.h"

#include OATPP_CODEGEN_BEGIN(DTO)

/**
 * 示例分页查询对象
 */
class XxxQuery : public PageQuery
{
    DTO_INIT(XxxQuery, PageQuery);

};

#include OATPP_CODEGEN_END(DTO)
#endif // !_HX_QUERY_
```

## domin-Query (非分页)

```C++
#pragma once
/*
 Copyright Zero One Star. All rights reserved.

 @Author: Heng_Xin
 @Date: $(time)

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/
#ifndef _HX_QUERY_
#define _HX_QUERY_

#include "../../GlobalInclude.h"

#include OATPP_CODEGEN_BEGIN(DTO)

/**
 * 示例查询对象
 */
class XxxQuery : public oatpp::DTO
{
    DTO_INIT(XxxQuery, DTO);

};

#include OATPP_CODEGEN_END(DTO)
#endif // !_HX_QUERY_
```


## domin-DTO (分页)

```C++
#pragma once
/*
 Copyright Zero One Star. All rights reserved.

 @Author: Heng_Xin
 @Date: $(time)

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/
#ifndef _HX_DTO_
#define _HX_DTO_
#include "../../GlobalInclude.h"

#include OATPP_CODEGEN_BEGIN(DTO)

/**
 * 示例传输对象
 */
class XxxDTO : public oatpp::DTO
{
    DTO_INIT(XxxDTO, DTO);
};

/**
 * 示例分页传输对象
 */
class XxxPageDTO : public PageDTO<XxxDTO::Wrapper> 
{
    DTO_INIT(XxxPageDTO, PageDTO<XxxDTO::Wrapper>);
};

#include OATPP_CODEGEN_END(DTO)
#endif // !_HX_DTO_
```

## domin-VO (分页)

```C++
#pragma once
/*
 Copyright Zero One Star. All rights reserved.

 @Author: Heng_Xin
 @Date: $(time)

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/
#ifndef _HX_VO_
#define _HX_VO_

#include "../../GlobalInclude.h"
#include "../../dto/xxx/XxxDTO.h"

#include OATPP_CODEGEN_BEGIN(DTO)

/**
 * 示例显示JsonVO，用于响应给客户端的Json对象
 */
class XxxJsonVO : public JsonVO<XxxDTO::Wrapper> {
    DTO_INIT(XxxJsonVO, JsonVO<XxxDTO::Wrapper>);
};

/**
 * 示例分页显示JsonVO，用于响应给客户端的Json对象
 */
class XxxPageJsonVO : public JsonVO<XxxPageDTO::Wrapper> {
    DTO_INIT(XxxPageJsonVO, JsonVO<XxxPageDTO::Wrapper>);
};

#include OATPP_CODEGEN_END(DTO)

#endif // !_HX_VO_
```

## controllr

```C++
#pragma once
/*
 Copyright Zero One Star. All rights reserved.

 @Author: Heng_Xin
 @Date: $(time)

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/
#ifndef _HX_CONTROLLER_
#define _HX_CONTROLLER_

#include "domain/vo/BaseJsonVO.h"
#include "domain/query/xxx/XxxQuery.h"
#include "domain/dto/xxx/XxxDTO.h"
#include "domain/vo/xxx/XxxVO.h"

#include OATPP_CODEGEN_BEGIN(ApiController)

/**
 * 示例控制器，演示基础接口的使用
 */
class XxxController : public oatpp::web::server::api::ApiController // 1 继承控制器
{
    // 2 定义控制器访问入口
    API_ACCESS_DECLARE(XxxController);
    // 3 定义接口
public:
    // 3.1 定义查询接口描述
    ENDPOINT_INFO(queryXxx) {
    // 定义接口标题
    API_DEF_ADD_TITLE(ZH_WORDS_GETTER("Xxx.get.summary"));
    // 定义默认授权参数（可选定义，如果定义了，下面ENDPOINT里面需要加入API_HANDLER_AUTH_PARAME）
    API_DEF_ADD_AUTH();
    // 定义响应参数格式
    API_DEF_ADD_RSP_JSON_WRAPPER(XxxPageJsonVO);
    // 定义分页查询参数描述
    API_DEF_ADD_PAGE_PARAMS();
    // 定义其他查询参数描述
    API_DEF_ADD_QUERY_PARAMS(String, "name", ZH_WORDS_GETTER("Xxx.field.name"), "li ming", false);
    API_DEF_ADD_QUERY_PARAMS(String, "sex", ZH_WORDS_GETTER("Xxx.field.sex"), "N", false);
    }
    // 3.2 定义查询接口处理
    ENDPOINT(API_M_GET, "/xxx", queryXxx, QUERIES(QueryParams, queryParams), API_HANDLER_AUTH_PARAME) {
        // 解析查询参数为Query领域模型
        API_HANDLER_QUERY_PARAM(userQuery, XxxQuery, queryParams);
        // 呼叫执行函数响应结果
        API_HANDLER_RESP_VO(execQueryXxx(userQuery, authObject->getPayload()));
    }
private:
    // 3.3 演示分页查询数据
    XxxPageJsonVO::Wrapper execQueryXxx(const XxxQuery::Wrapper& query, const PayloadDTO& payload);
};

#include OATPP_CODEGEN_END(ApiController)
#endif // _HX_CONTROLLER_
```

## Service

```C++
#pragma once
/*
 Copyright Zero One Star. All rights reserved.

 @Author: Heng_Xin
 @Date: $(time)

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/
#ifndef _HX_SERVICE_
#define _HX_SERVICE_
#include <list>
#include "domain/vo/xxx/XxxVO.h"
#include "domain/query/xxx/XxxQuery.h"
#include "domain/dto/xxx/XxxDTO.h"

/**
 * 示例服务实现，演示基础的示例服务实现
 */
class XxxService
{
public:
    // 分页查询所有数据
    XxxPageDTO::Wrapper listAll(const XxxQuery::Wrapper& query);
    // 保存数据
    uint64_t saveData(const XxxDTO::Wrapper& dto);
    // 修改数据
    bool updateData(const XxxDTO::Wrapper& dto);
    // 通过ID删除数据
    bool removeData(uint64_t id);
};

#endif // !_HX_SERVICE_
```

## DAO

```C++
#pragma once
/*
 Copyright Zero One Star. All rights reserved.

 @Author: Heng_Xin
 @Date: $(time)

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/
#ifndef _HX_DAO_
#define _HX_DAO_
#include "BaseDAO.h"
#include "../../domain/do/xxx/XxxDO.h"
#include "../../domain/query/xxx/XxxQuery.h"

/**
 * 示例表数据库操作实现
 */
class XxxDAO : public BaseDAO
{
public:
    // 统计数据条数
    uint64_t count(const XxxQuery::Wrapper& query);
    // 分页查询数据
    list<XxxDO> selectWithPage(const XxxQuery::Wrapper& query);
    // 通过姓名查询数据
    list<XxxDO> selectByName(const string& name);
    // 插入数据
    uint64_t insert(const XxxDO& iObj);
    // 修改数据
    int update(const XxxDO& uObj);
    // 通过ID删除数据
    int deleteById(uint64_t id);
};
#endif // !_HX_DAO_
```

```C++
/*
 * Copyright Zero One Star. All rights reserved.
 *
 * @Author: Heng_Xin
 * @Date: ${time}$
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * */
#include "stdafx.h"
#include <sstream>
```

## 数据库字段匹配

```C++
#pragma once
/*
 Copyright Zero One Star. All rights reserved.

 @Author: Heng_Xin
 @Date: $(time)

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

      https://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/
#ifndef _XXX_MAPPER_
#define _XXX_MAPPER_

#include "Mapper.h"
#include "../../domain/do/xxx/XxxDO.h"

/**
 * 示例表字段匹配映射
 */
class XxxMapper : public Mapper<XxxDO>
{
public:
    XxxDO mapper(ResultSet* resultSet) const override
    {
        XxxDO data;
        data.setId(resultSet->getUInt64(1)); // 第一个字段对应...
        data.setName(resultSet->getString(2));
        data.setSex(resultSet->getString(3));
        data.setAge(resultSet->getInt(4));
        return data;
    }
};

#endif // !_XXX_MAPPER_
```
