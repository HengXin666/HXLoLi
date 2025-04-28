# 项目实战: HX-ANiMe-Web
## 0. 创建数据库

```sql
CREATE DATABASE hx_anime CHARACTER SET utf8mb4 COLLATE utf8mb4_bin;
```

- 入库:

```sql
USE hx_anime;
```

## 1. 数据库表设计
### 1.1 用户
<details>
<summary>废弃的方案</summary>

> 废弃原因: 需要两次查询...

### 1.1.1 用户基础信息表

|uid(自增且唯一)|nickname(名称)|avatar(头像)|
|:-:|:-:|:-:|
|1|HX|.jpg|

### 1.1.2 用户授权信息表

采用一种一对多的方式, 即 uid - 登录方式 - 密码
</details>

### 1.1.1 用户基本信息表

```sql
CREATE TABLE `base_user` (
  `uid` bigint NOT NULL AUTO_INCREMENT COMMENT '用户ID',
  `type` tinyint(1) NOT NULL DEFAULT '2' COMMENT '用户类型: 1:管理员; 2:普通用户; 3:只读用户;',
  `user_name` varchar(100) CHARACTER SET utf8mb4 COLLATE utf8mb4_bin NOT NULL COMMENT '用户名(唯一)',
  `nickname` varchar(100) CHARACTER SET utf8mb4 COLLATE utf8mb4_bin NOT NULL COMMENT '昵称',
  `email` varchar(255) CHARACTER SET utf8mb4 COLLATE utf8mb4_bin NOT NULL UNIQUE COMMENT '用户邮箱',
  `password` varchar(200) CHARACTER SET utf8mb4 COLLATE utf8mb4_bin NOT NULL COMMENT '用户密码',
  `salt` varchar(200) CHARACTER SET utf8mb4 COLLATE utf8mb4_bin NOT NULL COMMENT '密码加盐',
  `avatar` varchar(255) CHARACTER SET utf8mb4 COLLATE utf8mb4_bin DEFAULT NULL COMMENT '用户头像',
  `last_login_time` datetime DEFAULT NULL COMMENT '最后登录时间',
  `cre_time` datetime NOT NULL DEFAULT CURRENT_TIMESTAMP COMMENT '创建时间',
  PRIMARY KEY (`uid`),
  INDEX `idx_email` (`email`) -- 为email添加索引, 需要保证email唯一
) ENGINE=InnoDB AUTO_INCREMENT=1 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_bin COMMENT='用户';
```

### 1.2 图的分用户存储

```C++
/**
 * 后端存储架构 (到时候字段名统一一下嗷)
 * 
 * 还需要一个用户id, 和用户的图id(一个用户可能有多个表) | <-- 摊牌了, 不搞企业级, 不搞该死的分库分表..
 * 
 * 结点表:
 *  |(内键)节点id|用户id(外键)|用户表id(外键)|图例id(外键)|名称|imgUrl|描述|
 *  - 注: 名称附带样式, 这样如果需要什么注音的也可以搞
 * 
 * 结点备注表:
 *  // 本表为灵活的数据增量适配, 因为是mysql而不是芒果db
 *  // 如果前端需要添加而外的项, 则先判断是否有这个项的key, 有则改(覆盖); 否则才插入. 
 *  |唯一自增id|用户id(外键)|用户表id(外键)|结点id(外键)|备注key|备注val|
 * 
 * 边集表:
 *  |(内键)边id|用户id(外键)|用户表id(外键)|fromNodeId|toNodeId|
 * 
 * 图表图例表:
 *  |(内键)图例id|用户id(外键)|用户表id(外键)|图例名称|图例颜色|
 * 
 * 用户图表表:
 *  |(内键)用户表id|用户id(外键)|图表名称|图表图标url|图表内容|
 * 
 * 查询过程:
 *  1. 用户登录并且进入到该图例画面
 *  2. 触发图表查询: 根据用户id, 查询到对应的图id(存储到本地, 前端展示信息), 并且默认打开id最小的图(或者不打开?) (如何防止用户篡改id而查询到其他用户的数据?)
 *  3. 如果打开图, 则触发查找:
 *      1) 根据图id、用户id, 查询到该图所有的图例
 *      2) 根据图id、用户id, 查询该图的所有节点和边 (并且动态加载: 后端一次查询, 分片返回/ 多次查询(分页游标查询), 次次返回)
 *      3) 前端同时构建
 * 
 * 创建与增加过程:
 *  - 创建图表 (创建即可(弹出窗口, 填写标题(必填), 内容, url 可空))
 *      | 前端传输时候配上 用户id 和 图 id 即可
 * 
 *  - 创建图例 (创建即可(弹出窗口, 填写名称(必填), 选择颜色))
 *      | 前端传输时候配上 用户id 和 图 id 即可
 * 
 *  - 添加节点 (选择添加节点所属的图例, 结点名称, 其他可空)
 *      | 前端传输时候配上 用户id 和 图 id 即可
 * 
 *  - 添加边 (是否可以通过左键点击一个节点作为开始, 右键一个节点作为终止, 变成一个有向节点)
 *      | 从而得到 fromNodeId 和 toNodeId
 *      | 前端传输时候配上 用户id 和 图 id 即可
 *      - 是否可以通过py调用爬虫接口实现? 只需要番名即可
 *          - py是否是本地实现对接?
 * 
 * 删除过程:
 *  - 删除结点, 传输结点id
 *      | 前端传输时候配上 用户id 和 图 id 即可
 *      - 还有边集, 也直接从后端`join`删除, 防止前端数据造假
 * 
 * - 删除边, 传输边id
 *      | 前端传输时候配上 用户id 和 图 id 即可
 * 
 * - 删除图例, 需要保证当前图上没有人使用这个图例 (前后端双重验证)
 *      | 前端传输时候配上 用户id 和 图 id 即可
 * 
 * - 删除图表, 先是请确定, 然后是, 要求用户输入密码进行确认 (当且仅当图为空的时候允许直接删除 (推荐用户使用改名))
 *      | 前端传输时候配上 用户id 和 图 id 即可
 * 
 * 修改过程:
 *  - 把添加的修改一下就ok
 * 
 * 备注:
 *  mysql分页游标: https://segmentfault.com/a/1190000042064994
 */
```

下面代码不要外键
```sql
-- 结点表
CREATE TABLE Nodes (
    node_id INT PRIMARY KEY AUTO_INCREMENT COMMENT '节点ID',
    user_id INT NOT NULL COMMENT '用户ID',
    user_table_id INT NOT NULL COMMENT '用户表ID',
    legend_id INT NOT NULL COMMENT '图例ID',
    name VARCHAR(255) NOT NULL COMMENT '节点名称',
    imgUrl VARCHAR(500) COMMENT '节点图片URL',
    description TEXT COMMENT '节点描述',
    FOREIGN KEY (user_id) REFERENCES Users(user_id),
    FOREIGN KEY (user_table_id) REFERENCES UserTables(user_table_id),
    FOREIGN KEY (legend_id) REFERENCES Legends(legend_id)
) ENGINE=InnoDB AUTO_INCREMENT=1 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_bin COMMENT '结点表';

-- 结点备注表
CREATE TABLE NodeNotes (
    note_id INT PRIMARY KEY AUTO_INCREMENT COMMENT '备注唯一自增ID',
    user_id INT NOT NULL COMMENT '用户ID',
    user_table_id INT NOT NULL COMMENT '用户表ID',
    node_id INT NOT NULL COMMENT '结点ID',
    note_key VARCHAR(255) NOT NULL COMMENT '备注键',
    note_value TEXT COMMENT '备注值',
    FOREIGN KEY (user_id) REFERENCES Users(user_id),
    FOREIGN KEY (user_table_id) REFERENCES UserTables(user_table_id),
    FOREIGN KEY (node_id) REFERENCES Nodes(node_id)
) ENGINE=InnoDB AUTO_INCREMENT=1 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_bin COMMENT '结点备注表';

-- 边集表
CREATE TABLE Edges (
    edge_id INT PRIMARY KEY AUTO_INCREMENT COMMENT '边ID',
    user_id INT NOT NULL COMMENT '用户ID',
    user_table_id INT NOT NULL COMMENT '用户表ID',
    from_node_id INT NOT NULL COMMENT '起始节点ID',
    to_node_id INT NOT NULL COMMENT '目标节点ID',
    FOREIGN KEY (user_id) REFERENCES Users(user_id),
    FOREIGN KEY (user_table_id) REFERENCES UserTables(user_table_id),
    FOREIGN KEY (from_node_id) REFERENCES Nodes(node_id),
    FOREIGN KEY (to_node_id) REFERENCES Nodes(node_id)
) ENGINE=InnoDB AUTO_INCREMENT=1 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_bin COMMENT '边集表';

-- 图表图例表
CREATE TABLE Legends (
    legend_id INT PRIMARY KEY AUTO_INCREMENT COMMENT '图例ID',
    user_id INT NOT NULL COMMENT '用户ID',
    user_table_id INT NOT NULL COMMENT '用户表ID',
    legend_name VARCHAR(255) NOT NULL COMMENT '图例名称',
    legend_color VARCHAR(50) NOT NULL COMMENT '图例颜色',
    FOREIGN KEY (user_id) REFERENCES Users(user_id),
    FOREIGN KEY (user_table_id) REFERENCES UserTables(user_table_id)
) ENGINE=InnoDB AUTO_INCREMENT=1 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_bin COMMENT '图表图例表';

-- 用户图表表
CREATE TABLE UserTables (
    user_table_id INT PRIMARY KEY AUTO_INCREMENT COMMENT '用户表ID',
    user_id INT NOT NULL COMMENT '用户ID',
    table_name VARCHAR(255) NOT NULL COMMENT '图表名称',
    icon_url VARCHAR(500) COMMENT '图表图标URL',
    table_content TEXT COMMENT '图表内容',
    FOREIGN KEY (user_id) REFERENCES Users(user_id)
) ENGINE=InnoDB AUTO_INCREMENT=1 DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_bin COMMENT '用户图表表';
```

## 2. token

用来做唯一识别的

## 3. vue 登录
### 3.1 状态管理
- 使用Vuex, 单例的存储信息, 方便在不同界面展示

```C++
// main.cpp文件
int sum(); // 声明

int main() {
  /// ...
  sum(); // 调用函数
  return 0;
}


// sum.cpp 文件
int sum() {
  // ...
}
```
