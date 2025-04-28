# 结构化查询语言
## SQL分类
结构化查询语句，英文名称为`Structured Query Language`，简称`SQL`。结构化查询语句分为数据定义语言、数据操作语言、数据查询语言和数据控制语言四大类。

|名称|描述|命令|
|:-:|:-:|:-:|
|数据定义语言<br>(DDL)|数据库、数据表的创建、修改和删除|CREATE、ALTER、DROP|
|数据操作语言<br>(DML)|数据的增加、修改和删除|INSERT、UPDATE、DELETE|
|数据查询语言<br>(DQL)|数据的查询|SELECT|
|数据控制语言<br>(DCL)|用户授权、事务的提交和回滚|GRANT、COMMIT、ROLLBACK|

## 数据库操作
### 创建数据库的语法
```sql
-- [] 是可选的意思, 实际写的时候不用写 "[]"
CREATE DATABASE [IF NOT EXISTS] 数据库名称 DEFAULT CHARACTER SET 字符集 COLLATE 排序规则;
```

其中的 `[IF NOT EXISTS]` 的意思是: 如果不存在 则创建

示例: 创建数据库`hx_demp`，并指定字符集为`GBK`，排序规则为`GBK_CHINESE_CI`

注: `--` 是注释, `.sql`为后缀的文件是数据库指令文件
```sql
-- 创建数据库
mysql> CREATE DATABASE IF NOT EXISTS hx_demo DEFAULT CHARACTER SET GBK COLLATE GBK_CHINESE_CI;
Query OK, 1 row affected (0.01 sec)
```

### 修改数据库的语法

```sql
ALTER DATABASE 数据库名称 CHARACTER SET 字符集 COLLATE 排序规则;
```

示例：修改数据库`hx_demp`的字符集为`UTF-8`，排序规则为`UTF8_GENERAL_CI`

```sql
-- 修改数据库字符集为UTF8, 排序规则为 UTF8_GENERAL_CI
mysql> ALTER DATABASE hx_demo CHARACTER SET UTF8 COLLATE UTF8_GENERAL_CI;
Query OK, 1 row affected, 2 warnings (0.01 sec)
```

### 删除数据库的语法

```sql
-- []是可选的, 意思是 如果存在则删除
DROP DATABASE [IF EXISTS] 数据库名称;
```

示例：删除数据库`hx_demp`

```sql
-- 删除数据库
mysql> DROP DATABASE IF EXISTS hx_demo;
Query OK, 0 rows affected (0.01 sec)
```

### 查看数据库语法

```sql
SHOW DATABASES;
```

示例:

```sql
-- 安装mysql后, 会存在4个默认的数据库
mysql> SHOW DATABASES;
+--------------------+
| Database           |
+--------------------+
| information_schema |
| mysql              |
| performance_schema |
| sys                |
+--------------------+
4 rows in set (0.01 sec)
```

### 使用数据库的语法

```sql
USE 数据库名称;

-- 注: 当没有指定操作对象时，默认情况下，MySQL将会操作当前连接的默认数据库。
-- 这个默认数据库是在连接到MySQL服务器时指定的，通常是在连接命令中使用-D参数或者通过USE语句来设置的。
-- 如果没有明确指定，默认情况下新建的连接不会指定默认数据库，因此使用任何需要操作数据库的命令之前都应先使用 USE 数据库名称; 来指定要操作的数据库。
```

示例: 使用`hx_demo`数据库

```sql
-- 切换数据库
mysql> USE hx_demo;
Database changed
```

##  列类型
在`MySQL`中，常用列类型主要分为数值类型、日期时间类型、字符串类型。
### 数值类型
|类型|说明|取值范围|存储需求|
|:-:|:-:|:-|:-:|
|tinyint|非常小的数据|有符号值: $-2^7～2^7-1$ <br>无符号值: $0～2^8-1$|1字节|
|smallint|较小的数据|有符号值: $-2^{15}～2^{15}-1$ <br>无符号值: $0$ ~ $2^{16}-1$|2字节|
|mediumint|中等大小的数据|有符号值: $-2^{23}～2^{23}-1$ <br>无符号值: $0$ ~ $2^{24}-1$|3字节|
|int|标准整数|有符号值: $-2^{31}～2^{31}-1$ <br>无符号值: $0$ ~ $2^{32}-1$|4字节|
|bigint|较大的整数|有符号值: $-2^{63}$ ~ $2^{63}-1$ <br>无符号值: $0$ ~ $2^{64}-1$|8字节|
float|单精度浮点数|无符号值: $1.1754351 *10^{-38}~3.402823466 *10^{38}$|4字节|
|double|双精度浮点数|无符号值: $2.22507385*10^{-308}～1.79769313*10^{308}$|8字节|
|decimal|字符串形式的浮点数|decimal(m, d)|m个字节|
### 日期时间类型
|类型|说明|取值范围|
|---|---|---|
|DATE|日期|1000-01-01 ~ 9999-12-31|
|TIME|时间|00:00:00 ~ 23:59:59|
|DATETIME|日期时间|1000-01-01 00:00:00 ~ 9999-12-31 23:59:59|
|TIMESTAMP|`YYYY-MM-dd HH:mm:ss`格式表示的时间戳|1970-01-01 00:00:00 ~ 2038-01-19 03:14:07|
|YEAR|年份|1901 ~ 2155|
### 字符串类型
|类型|说明|最大长度|
|---|---|---|
|`CHAR`[(M)]|固定长字符串，检索快但费空间，`0 <= M <= 255`|M个字符|
|`VARCHAR`[(M)]|变长字符串，检索慢但省空间，`0 <= M <= 65535`|变长度|
|`TEXT`|最大长度为65535个字符|$2^{16}-1$字节|
|`MEDIUMTEXT`|最大长度为16777215个字符|$2^{24}-1$字节|
|`LONGTEXT`|最大长度为4294967295个字符|$2^{32}-1$字节|

### 列类型修饰属性
|属性名|说明|示例|
|---|---|---|
|UNSIGNED|无符号，只能修来修饰数值类型，表名该列数据不能出现负数|INT(4) UNSIGNED，表示只能为4位大于等于0的整数|
|ZEROFILL|自动补零，只能修来修饰数值类型，表名该列数据前面的数字为0|INT(4) ZEROFILL，表示4位的整数，前面的数字为0，如`13`则为`0013`|
|NOT NULL|非空，只能修来修饰数值类型，表名该列数据不能为NULL|VARCHAR (20) NOT NULL，表示该列数据不能为空值|
|DEFAULT|默认值，只能修来修饰数值类型，表名该列数据默认值为该值|INT(4) DEFAULT 123，表示4位的整数，默认值为123|
|AUTO_INCREMENT|表示自增长，只能应用于数值列类型，该列类型必须为键，且不能为空|INT(11) AUTO_INCREMENT NOT NULL PRIMARY KEY。第一次为该列中插入值时为1，第二次为2|

## 数据表操作
### 数据表类型
MySQL中的数据表类型有许多，如 MyISAM、InnoDB、HEAP、BOB、CSV 等。其中最常用的就是`MyISAM`和`InnoDB`.
#### MyISAM 与 InnoDB 的区别
|名称|MyISAM|InnoDB|
|---|---|---|
|事务处理|不支持|支持|
|数据行锁定|不支持|支持|
|外键约束|不支持|支持|
|全文索引|支持|不支持|
|表空间大小|较小|较大，约`MyISAM`的2倍|

- 事务处理：
    - MyISAM：不支持事务，这意味着一旦发生错误或中断操作，无法通过回滚来恢复到事务开始前的状态。
    - InnoDB：支持事务处理，满足ACID（原子性、一致性、隔离性和持久性）属性，适合需要复杂事务管理的场景。
- 数据行锁定：(类似于多线程中的`锁`)
    - MyISAM：不支持行级锁定，只支持表级锁定，在并发环境下可能导致更多的阻塞和等待，影响性能。
    - InnoDB：支持行级锁定，允许在多用户同时访问时对数据进行更细粒度的锁定控制，提高了并发读写效率。
- 外键约束：
    - MyISAM：不支持外键约束，因此不能强制执行参照完整性规则。
    - InnoDB：支持外键约束，可以确保数据库中相关表之间的数据一致性。
- 全文索引：
    - MyISAM：支持全文索引，适合于需要进行大量文本内容搜索的应用场景。
    - InnoDB：早期版本不支持全文索引，但在MySQL 5.6.4版本以后也引入了全文索引功能，不过对于较旧版本可能仍需要依赖MyISAM或其他解决方案。

##### 如何选择数据表的类型?
当涉及的业务操作以查询居多，修改和删除较少时，可以使用`MyISAM`。当涉及的业务操作经常会有修改和删除操作时，使用`InnoDB`。

### 创建数据表
语法:

```sql
CREATE TABLE [IF NOT EXISTS] 数据表名称(
    字段名1 列类型(长度) [修饰属性] [键/索引] [注释],
    字段名2 列类型(长度) [修饰属性] [键/索引] [注释],
    字段名3 列类型(长度) [修饰属性] [键/索引] [注释],
    ......
    字段名n 列类型(长度) [修饰属性] [键/索引] [注释]
) [ENGINE = 数据表类型][CHARSET=字符集编码] [COMMENT=注释];
```

示例：创建学生表，表中有字段学号、姓名、性别、年龄和成绩

```sql
mysql> USE hx_demo
Database changed
mysql> CREATE TABLE IF NOT EXISTS student (
    ->     `number` VARCHAR(30) NOT NULL PRIMARY KEY COMMENT '学号, 主键',
    ->     name VARCHAR(30) NOT NULL COMMENT '姓名',
    ->     sex TINYINT(1) UNSIGNED DEFAULT 0 COMMENT '性别: 0 为男, 1 为女, 2-其他',
    ->     age TINYINT(3) UNSIGNED DEFAULT 0 COMMENT '年龄',
    ->     score DOUBLE(5, 2) UNSIGNED COMMENT '成绩'
    -> )ENGINE=InnoDB CHARSET=UTF8 COMMENT='学生信息表';
Query OK, 0 rows affected, 5 warnings (0.07 sec)
```

注: `` 符号（反引号）在MySQL中用于包围数据库对象的名称，如表名、列名等。其作用是标识这些字符串为SQL中的标识符，并且在标识符中允许包含特殊字符或与SQL保留关键字冲突的名称。在MySQL中，尽管不强制要求对所有标识符使用反引号，但在处理包含空格、特殊字符或者与SQL关键字相同的标识符时，使用反引号可以确保正确解析。

### 查
#### 列出该数据库的所有表

```sql
SHOW TABLES;
```

示例：

```sql
mysql> SHOW TABLES;
+-------------------+
| Tables_in_hx_demo |
+-------------------+
| stu               |
+-------------------+
1 row in set (0.01 sec)
```

#### 查看表结构
```sql
DESC 表名;

SHOW COLUMNS FROM 表名; -- 查看表的所有字段(同上)
```

示例:

```sql
mysql> DESC stu;
+--------+----------------------+------+-----+---------+-------+
| Field  | Type                 | Null | Key | Default | Extra |
+--------+----------------------+------+-----+---------+-------+
| number | varchar(30)          | NO   | PRI | NULL    |       |
| name   | varchar(30)          | NO   |     | NULL    |       |
| sex    | tinyint unsigned     | YES  |     | 0       |       |
| age    | tinyint unsigned     | YES  |     | 0       |       |
| score  | double(5,2) unsigned | YES  |     | NULL    |       |
| phone  | varchar(11)          | NO   |     | NULL    |       |
+--------+----------------------+------+-----+---------+-------+
6 rows in set (0.02 sec)
```


### 修改数据表
#### 修改表名

```sql
ALTER TABLE 表名 RENAME AS 新表名;
```

示例：将student表名称修改为 stu

```sql
mysql> ALTER TABLE student RENAME AS stu;
Query OK, 0 rows affected (0.03 sec)
```

#### 增加字段

```sql
ALTER TABLE 表名 ADD 字段名 列类型(长度) [修饰属性] [键/索引] [注释];
```

示例：在 stu 表中添加字段联系电话(phone)，类型为字符串，长度为11，非空


```sql
mysql> ALTER TABLE stu ADD phone VARCHAR(11) NOT NULL COMMENT '联系电话';
Query OK, 0 rows affected (0.02 sec)
Records: 0  Duplicates: 0  Warnings: 0
```

#### 修改字段

```sql
-- MODIFY 只能修改字段的修饰属性
ALTER TABLE 表名 MODIFY 字段名 列类型(长度) [修饰属性] [键/索引] [注释];

-- CHANGE 可以修改字段的名字以及修饰属性
ALTER TABLE 表名 CHANGE 字段名 新字段名 列类型(长度) [修饰属性] [键/索引] [注释];
```

示例： 将 stu 表中的 sex 字段的类型设置为 VARCHAR ，长度为2，默认值为'男'，注释为 "性别，男，女，其他"

```sql
mysql> ALTER TABLE stu MODIFY sex VARCHAR(2) DEFAULT '男' COMMENT '性别，男，女，其他';
Query OK, 0 rows affected (0.05 sec)
Records: 0  Duplicates: 0  Warnings: 0
```

示例：将 stu 表中 phone 字段修改为 mobile ，属性保持不变

```sql
mysql> ALTER TABLE stu CHANGE phone mobile VARCHAR(11) NOT NULL COMMENT '手机号';
Query OK, 0 rows affected (0.01 sec)
Records: 0  Duplicates: 0  Warnings: 0
```

#### 删除字段

```sql
ALTER TABLE 表名 DROP 字段名;
```

示例：将 stu 表中的 mobile 字段删除

```sql
mysql> ALTER TABLE stu DROP mobile;
Query OK, 0 rows affected (0.03 sec)
Records: 0  Duplicates: 0  Warnings: 0
```

### 删除数据表

```sql
DROP TABLE [IF EXISTS] 表名;
```

示例：删除数据表 stu

```sql
mysql> DROP TABLE IF EXISTS stu;
Query OK, 0 rows affected (0.02 sec)
```
