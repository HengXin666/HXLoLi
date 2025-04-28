# DML语句
**DML全称为Data Manipulation Language**，表示数据操作语言。主要体现于对表数据的增删改操作。因此`DML`仅包括`INSERT`、`UPDATE`和`DELEETE`语句。

本文使用这个表作为演示:

```sql
-- 创建课程表 编号, 课程名称, 学分, 学时
CREATE TABLE IF NOT EXISTS course (
    `number` INT(11) AUTO_INCREMENT NOT NULL PRIMARY KEY COMMENT '编号, 主键',
    name VARCHAR(30) NOT NULL COMMENT '课程名称',
    credit DOUBLE(2, 1) UNSIGNED DEFAULT 0 COMMENT '学分',
    hour TINYINT(3) UNSIGNED DEFAULT 0 COMMENT '学时'
)ENGINE=InnoDB CHARSET=UTF8 COMMENT='课程信息表';
```

## INSERT语句

```sql
-- 需要注意，VALUES后的字段值必须与表名后的字段名一一对应
INSERT INTO 表名(字段名1, 字段名2, ..., 字段名n) VALUES(字段值1, 字段值2, ..., 字段值n);
-- 需要注意，VALUES后的字段值必须与创建表时的字段顺序保持一一对应
INSERT INTO 表名 VALUES(字段值1, 字段值2, ..., 字段值n);
-- 一次性插入多条数据
INSERT INTO 表名(字段名1, 字段名2, ..., 字段名n) VALUES(字段值1, 字段值2, ..., 字段值n),(字段值1, 字段值2, ..., 字段值n), ... , (字段值1, 字段值2, ..., 字段值n);
INSERT INTO 表名 VALUES(字段值1, 字段值2, ..., 字段值n), (字段值1, 字段值2, ..., 字段值n), ..., (字段值1, 字段值2, ..., 字段值n);
```

示例: 向课程表中插入数据

```sql
-- 向课程表中插入数据
INSERT INTO course (number, name, credit, hour) VALUES (1, 'C语言程序设计', 2, 40);

-- 可以按照指定字段的顺序插入数据, 而不是创建时候的顺序
INSERT INTO course (number, credit, name, hour) VALUES (2, 2, 'HTML', 2, 20);

-- 不指定字段的顺序, 则按照创建时候的顺序插入数据
INSERT INTO course VALUES (3, 'C++程序设计', 3, 40);

-- 可以一次插入多条数据
INSERT INTO course (number, name, credit, hour) VALUES (4, 'Java程序设计', 3, 40), (5, '数据库原理', 3, 40);

-- 如果不指定键值, 则自动生成键值, 前提是主键必须是自增的(AUTO_INCREMENT)
INSERT INTO course (name, credit, hour) VALUES ('数据结构', 3, 40), ('操作系统', 2, 20);
```

> ## REPLACE语句
>
> replace into 跟 insert into功能类似，不同点在于：replace into 首先尝试插入数据到表中
>
> 1. 如果发现表中已经有此行数据（根据主键或者唯一索引判断）则先删除此行数据，然后插入新的数据；
> 
> 2. 否则，直接插入新数据。
>
> 要注意的是: 插入数据的表必须有主键或者是唯一索引！否则的话，replace into 会直接插入数据，这将导致表中出现重复的数据。 

## UPDATE语句

```sql
UPDATE 表名 SET 字段名1=字段值1[,字段名2=字段值2, ..., 字段名n=字段值n] [WHERE 修改条件]
```

### WHERE条件子句
在Java中，条件的表示通常都是使用关系运算符来表示，在SQL语句中也是一样，使用 >, <, >=, <=, != 来表示。不同的是，除此之外，SQL中还可以使用SQL专用的关键字来表示条件。这些将在后面的DQL语句中详细讲解。

在Java中，条件之间的衔接通常都是使用逻辑运算符来表示，在SQL语句中也是一样，但通常使用AND来表示逻辑与(&&)，使用OR来表示逻辑或(||)

示例:

```sql
WHERE time > 20 && time < 40; 

-- 等价于
WHERE time > 20 AND time <40;
```

### UPDATE语句
示例: 将数据库的学分更改为4，学时更改为15

```sql
-- 将数据库的学分更改为4，学时更改为15
UPDATE course SET credit = 4, hour = 15 WHERE name = '数据库原理';

-- 注意: sql中 '=' 不是赋值 而是代表 '==' 判断语句!
```

## DELETE语句

```sql
DELETE FROM 表名 [WHERE 删除条件];

-- 注: 如果不写条件就是全部删除咯
```

示例: 删除课程表中课程编号为1的数据
```sql
-- 删除课程表中课程编号为1的数据
DELETE FROM course WHERE number = 1;
```

## TRUNCATE语句

```sql
-- 清空表中数据
TRUNCATE [TABLE] 表名;
```

示例: 清空课程表数据

```sql
TRUNCATE course;
```

### DELETE与TRUNCATE区别
- DELETE语句根据条件删除表中数据，而TRUNCATE语句则是将表中数据全部清空；如果DELETE语句要删除表中所有数据，那么在效率上要低于TRUNCATE语句。

- 如果表中有自增长列，TRUNCATE语句会重置自增长的计数器，但DELETE语句不会。

- TRUNCATE语句执行后，数据无法恢复，而DELETE语句执行后，可以使用事务回滚进行恢复。

- **物理索引结构**：`TRUNCATE TABLE`实际上是删除了原有的表，并重新创建了一个相同结构的空表。因此，原有的索引也会被删除并重新创建。所以，从这个角度来看，可以认为是“重置”了物理索引，因为原有的索引数据结构被移除，新的索引结构在重新创建的表上建立。但需要注意的是，虽然索引被重建，索引的定义（例如，哪些列被索引）保持不变。

# DQL语句
DQL全称是**Data Query Language**，表示数据查询语言。体现在数据的查询操作上，因此，DQL仅包括`SELECT`语句。

## SELECT语句

```sql
SELECT ALL/DISTINCT * | 字段名1 AS 别名1[,字段名1 AS 别名1, ..., 字段名n AS 别名n] FROM 表名 WHERE 查询条件;
```

解释说明:
- `ALL`表示查询所有满足条件的记录，默认就是此项可以省略;
- `DISTINCT`表示去掉查询结果中重复的记录
- `AS`可以给数据列、数据表取一个别名

使用示例:

```sql
-- 查询全部
mysql> SELECT * FROM course;
+--------+--------------+--------+------+
| number | name         | credit | hour |
+--------+--------------+--------+------+
|      3 | C++程序设计  |    3.0 |   40 |
|      4 | Java程序设计 |    3.0 |   40 |
|      5 | 数据库原理   |    4.0 |   15 |
|      6 | 数据结构     |    3.0 |   40 |
|      7 | 操作系统     |    2.0 |   20 |
+--------+--------------+--------+------+
5 rows in set (0.00 sec)

-- 等价于
mysql> SELECT all * FROM course;
+--------+--------------+--------+------+
| number | name         | credit | hour |
+--------+--------------+--------+------+
|      3 | C++程序设计  |    3.0 |   40 |
|      4 | Java程序设计 |    3.0 |   40 |
|      5 | 数据库原理   |    4.0 |   15 |
|      6 | 数据结构     |    3.0 |   40 |
|      7 | 操作系统     |    2.0 |   20 |
+--------+--------------+--------+------+
5 rows in set (0.00 sec)

-- 查询指定字段+字段起别名+表起别名, 注意顺序也是列出来那样
mysql> SELECT name AS '课程名称', number FROM course AS 课程库;
+--------------+--------+
| 课程名称     | number |
+--------------+--------+
| C++程序设计  |      3 |
| Java程序设计 |      4 |
| 数据库原理   |      5 |
| 数据结构     |      6 |
| 操作系统     |      7 |
+--------------+--------+
5 rows in set (0.00 sec)

-- 查询学时(起别名的AS可以省略), 并且不能重复
mysql> SELECT DISTINCT hour '学时' FROM course;
+------+
| 学时 |
+------+
|   40 |
|   15 |
|   20 |
+------+
3 rows in set (0.00 sec)

-- 注意: 不能重复是指整一个查询出来的数据不能重复, 而不是分别的每一个字段查询的数据
mysql> SELECT DISTINCT name '课程名称', hour '学时' FROM course;
+--------------+------+
| 课程名称     | 学时 |
+--------------+------+
| C++程序设计  |   40 |
| Java程序设计 |   40 |
| 数据库原理   |   15 |
| 数据结构     |   40 |
| 操作系统     |   20 |
+--------------+------+
5 rows in set (0.00 sec)
```


示例: 从课程表中查询课程编号小于5的课程名称

```sql
mysql> SELECT name FROM course WHERE `number` < 5;
+--------------+
| name         |
+--------------+
| C++程序设计  |
| Java程序设计 |
+--------------+
2 rows in set (0.00 sec)
```

从课程表中查询课程名称为"Java程序设计"的学分和学时

```sql
mysql> SELECT name ,hour `time` FROM course WHERE name='Java程序设计';
+--------------+------+
| name         | time |
+--------------+------+
| Java程序设计 |   40 |
+--------------+------+
1 row in set (0.00 sec)
```

## 比较操作符
|操作符|语法|说明|
|---|---|---|
|`NOT`|`NOT 其他表达式`(可以放下面的表达式)|如果表达式不满足，则条件满足. (取反)|
|`IS NULL`|`字段名 IS NULL`|如果字段的值为`NULL`, 则条件满足|
|`BETWEEN`|`字段名 BETWEEN 值1 AND 值2`|如果字段的值在`值1`和`值2`之间，条件满足|
|`LIKE`|`字段名 LIKE '%值%'`|在匹配'值', 前有`%`则代表前面需要有内容, 后面有`%`则代表后面有内容|
|`IN`|`字段名 IN (值1, 值2, .., 值n)`|如果字段的值在`值1`、`值2`...`值n`之中，条件满足|


```sql
-- 示例：从课程表查询课程名为NULL的课程信息
mysql> SELECT * FROM course WHERE name IS NULL;
Empty set (0.00 sec)

-- 示例：从课程表查询课程名不为NULL的课程信息
mysql> SELECT * FROM course WHERE name IS NOT NULL;
+--------+--------------+--------+------+
| number | name         | credit | hour |
+--------+--------------+--------+------+
|      3 | C++程序设计  |    3.0 |   40 |
|      4 | Java程序设计 |    3.0 |   40 |
|      5 | 数据库原理   |    4.0 |   15 |
|      6 | 数据结构     |    3.0 |   40 |
|      7 | 操作系统     |    2.0 |   20 |
+--------+--------------+--------+------+
5 rows in set (0.00 sec)

-- 示例：从课程表查询学分在2~4之间的课程信息 (实际上 >= <= > < 更加常用)
mysql> SELECT * FROM course WHERE credit BETWEEN 2 AND 4;
+--------+--------------+--------+------+
| number | name         | credit | hour |
+--------+--------------+--------+------+
|      3 | C++程序设计  |    3.0 |   40 |
|      4 | Java程序设计 |    3.0 |   40 |
|      5 | 数据库原理   |    4.0 |   15 |
|      6 | 数据结构     |    3.0 |   40 |
|      7 | 操作系统     |    2.0 |   20 |
+--------+--------------+--------+------+
5 rows in set (0.00 sec)

-- 示例：从课程表查询课程名包含"V"的课程信息
mysql> SELECT * FROM course WHERE name LIKE '%v%';
+--------+--------------+--------+------+
| number | name         | credit | hour |
+--------+--------------+--------+------+
|      4 | Java程序设计 |    3.0 |   40 |
+--------+--------------+--------+------+
1 row in set (0.00 sec)

-- 示例：从课程表查询课程名以"J"开头的课程信息, 那当然 '%x'就是以x结尾的啦
mysql> SELECT * FROM course WHERE name LIKE 'J%';
+--------+--------------+--------+------+
| number | name         | credit | hour |
+--------+--------------+--------+------+
|      4 | Java程序设计 |    3.0 |   40 |
+--------+--------------+--------+------+
1 row in set (0.00 sec)

-- 示例：从课程表查询课程编号为1,3,5的课程信息 (没有只是找不到, 并不会报错)
mysql> SELECT * FROM course WHERE `number` IN (1, 3, 5);
+--------+-------------+--------+------+
| number | name        | credit | hour |
+--------+-------------+--------+------+
|      3 | C++程序设计 |    3.0 |   40 |
|      5 | 数据库原理  |    4.0 |   15 |
+--------+-------------+--------+------+
2 rows in set (0.00 sec)
```

## 分组
数据表准备：新建学生表student，包含字段学号（no），类型为长整数，长度为20，是主键，自增长，非空；姓名（name），类型为字符串，长度为20，非空；性别（sex），类型为字符串，长度为2，默认值为"男"；年龄（age），类型为整数，长度为3，默认值为0；成绩（score），类型为浮点数，长度为5，小数点后面保留2位有效数字

```sql
CREATE TABLE IF NOT EXISTS student (
    `no` INT(20) AUTO_INCREMENT NOT NULL PRIMARY KEY COMMENT '学号, 主键',
    name VARCHAR(20) NOT NULL COMMENT '姓名',
    sex VARCHAR(2) DEFAULT '男' COMMENT '性别',
    age TINYINT(3) UNSIGNED DEFAULT 0 COMMENT '年龄',
    score DOUBLE(5, 2) UNSIGNED COMMENT '成绩'
)ENGINE=InnoDB CHARSET=UTF8 COMMENT='学生信息表';

INSERT INTO student (no, name, sex, age, score) VALUES
(DEFAULT, '张三', '男', 20, 59),
(DEFAULT, '李四', '女', 19, 62),
(DEFAULT, '王五', '其他', 21, 62),
(DEFAULT, '龙华', '男', 22, 75),
(DEFAULT, '金凤', '女', 18, 80),
(DEFAULT, '张华', '其他', 27, 88),
(DEFAULT, '李刚', '男', 30, 88),
(DEFAULT, '潘玉明', '女', 28, 81),
(DEFAULT, '凤飞飞', '其他', 32, 90);

mysql> SELECT * FROM student;
+----+--------+------+------+-------+
| no | name   | sex  | age  | score |
+----+--------+------+------+-------+
|  1 | 张三   | 男   |   20 | 59.00 |
|  2 | 李四   | 女   |   19 | 62.00 |
|  3 | 王五   | 其他 |   21 | 62.00 |
|  4 | 龙华   | 男   |   22 | 75.00 |
|  5 | 金凤   | 女   |   18 | 80.00 |
|  6 | 张华   | 其他 |   27 | 88.00 |
|  7 | 李刚   | 男   |   30 | 88.00 |
|  8 | 潘玉明 | 女   |   28 | 81.00 |
|  9 | 凤飞飞 | 其他 |   32 | 90.00 |
+----+--------+------+------+-------+
9 rows in set (0.00 sec)
```

### 分组查询

```sql
SELECT ALL/DISTINCT * | 字段名1 AS 别名1[,字段名1 AS 别名1, ..., 字段名n AS 别名n] FROM 表名 WHERE 查询条件 GROUP BY 字段名1[，字段名2,..., 字段名n]
```
分组查询所得的结果只是该组中的第一条数据。

示例: 从学生表查询成绩在80分以上的学生信息并按性别分组

```sql
mysql> SELECT * FROM student WHERE score > 80 GROUP BY sex;
+----+--------+------+------+-------+
| no | name   | sex  | age  | score |
+----+--------+------+------+-------+
|  6 | 张华   | 其他 |   27 | 88.00 |
|  7 | 李刚   | 男   |   30 | 88.00 |
|  8 | 潘玉明 | 女   |   28 | 81.00 |
+----+--------+------+------+-------+
3 rows in set (0.00 sec)
```

示例：从学生表查询成绩在60~80之间的学生信息并按性别和年龄分组

```sql
-- 先按照分组1进行分, 如果分组1有多个相同的, 则按照分组2来, 以此类推
mysql> SELECT * FROM student WHERE score >= 60 AND score <= 80 GROUP BY sex, age;
+----+------+------+------+-------+
| no | name | sex  | age  | score |
+----+------+------+------+-------+
|  2 | 李四 | 女   |   19 | 62.00 |
|  3 | 王五 | 其他 |   21 | 62.00 |
|  4 | 龙华 | 男   |   22 | 75.00 |
|  5 | 金凤 | 女   |   18 | 80.00 |
+----+------+------+------+-------+
4 rows in set (0.00 sec)
```

### 聚合函数
> [!TIP]
> where字句无法与聚合函数一起使用。因为where子句的运行顺序排在第二，运行到where时，表还没有被分组。
#### COUNT()
- **统计满足条件的数据总条数**

示例：从学生表查询成绩在80分以上的学生人数

```sql
mysql> SELECT COUNT(*) FROM student WHERE score > 80;
+----------+
| COUNT(*) |
+----------+
|        4 |
+----------+
1 row in set (0.00 sec)

-- 可以起个别名
mysql> SELECT COUNT(*) '统计结果' FROM student WHERE score > 80;
+----------+
| 统计结果 |
+----------+
|        4 |
+----------+
1 row in set (0.00 sec)
```
#### SUM()
- **只能用于数值类型的字段或者表达式，计算该满足条件的字段值的总和**

示例：从学生表查询成绩在80分以上的学生人数

```sql
mysql> SELECT SUM(score) 'SUM结果' FROM student WHERE score > 80;
+---------+
| SUM结果 |
+---------+
|  347.00 |
+---------+
1 row in set (0.00 sec)
```

#### AVG()
- **只能用于数值类型的字段或者表达式，计算该满足条件的字段值的平均值**

示例：从学生表查询男生、女生、其他类型的学生的平均成绩

```sql
mysql> SELECT sex, AVG(score) FROM student GROUP BY sex;
+------+------------+
| sex  | AVG(score) |
+------+------------+
| 男   |  74.000000 |
| 女   |  74.333333 |
| 其他 |  80.000000 |
+------+------------+
3 rows in set (0.00 sec)
```

#### MAX()
- **只能用于数值类型的字段或者表达式，计算该满足条件的字段值的最大值**

示例：从学生表查询学生的最大年龄

```sql
mysql> SELECT MAX(age) FROM student;
+----------+
| MAX(age) |
+----------+
|       32 |
+----------+
1 row in set (0.00 sec)
```

#### MIN()
- **只能用于数值类型的字段或者表达式，计算该满足条件的字段值的最小值**

示例：从学生表查询学生的最低分

```sql
mysql> SELECT MIN(score) FROM student;
+------------+
| MIN(score) |
+------------+
|      59.00 |
+------------+
1 row in set (0.00 sec)
```

### 分组查询结果筛选

分组后如果还需要满足其他条件，则需要使用`HAVING`子句来完成。

```sql
SELECT ALL/DISTINCT * | 字段名1 AS 别名1[,字段名1 AS 别名1, ..., 字段名n AS 别名n] FROM 表名 WHERE 查询条件 GROUP BY 字段名1[，字段名2,..., 字段名n] HAVING 筛选条件
```

示例：从学生表查询年龄在20~30之间的学生信息并按性别分组，找出组内平均分在74分以上的组

```sql
mysql> SELECT sex '组别', AVG(score) FROM student WHERE age >= 20 AND age <= 30 GROUP BY sex HAVING AVG(score) > 74;
+------+------------+
| 组别 | AVG(score) |
+------+------------+
| 其他 |  75.000000 |
| 女   |  81.000000 |
+------+------------+
2 rows in set (0.00 sec)
```

## 排序

```sql
SELECT ALL/DISTINCT * | 字段名1 AS 别名1[,字段名1 AS 别名1, ..., 字段名n AS 别名n] FROM 表名 WHERE 查询条件 ORDER BY 字段名1 ASC|DESC[，字段名2 ASC|DESC,..., 字段名n ASC|DESC]
```
`ORDER BY`必须位于`WHERE`条件之后。

- `ASC`: 从低到高
- `DESC`: 从高到低

示例：从学生表查询年龄在18~30岁之间的学生信息并按成绩从高到低排列，如果成绩相同，则按年龄从小到大排列

```sql
mysql> SELECT * FROM student WHERE age >= 18 AND age <= 30 ORDER BY score DESC, age ASC;
+----+--------+------+------+-------+
| no | name   | sex  | age  | score |
+----+--------+------+------+-------+
|  6 | 张华   | 其他 |   27 | 88.00 |
|  7 | 李刚   | 男   |   30 | 88.00 |
|  8 | 潘玉明 | 女   |   28 | 81.00 |
|  5 | 金凤   | 女   |   18 | 80.00 |
|  4 | 龙华   | 男   |   22 | 75.00 |
|  2 | 李四   | 女   |   19 | 62.00 |
|  3 | 王五   | 其他 |   21 | 62.00 |
|  1 | 张三   | 男   |   20 | 59.00 |
+----+--------+------+------+-------+
8 rows in set (0.00 sec)
```

## 分页

```sql
SELECT ALL/DISTINCT * | 字段名1 AS 别名1[,字段名1 AS 别名1, ..., 字段名n AS 别名n] FROM 表名 WHERE 查询条件 LIMIT 偏移量, 查询条数
```
- `LIMIT`的第一个参数表示`偏移量`，也就是**跳过的行数**。(从0开始算, 0是自己)
- `LIMIT`的第二个参数表示查询**返回的最大行数**，可能没有给定的数量那么多行。

示例：从学生表分页查询成绩及格的学生信息，每页显示3条，查询第2页学生信息

```sql
mysql> SELECT * FROM student LIMIT 3, 3;
+----+------+------+------+-------+
| no | name | sex  | age  | score |
+----+------+------+------+-------+
|  4 | 龙华 | 男   |   22 | 75.00 |
|  5 | 金凤 | 女   |   18 | 80.00 |
|  6 | 张华 | 其他 |   27 | 88.00 |
+----+------+------+------+-------+
3 rows in set (0.00 sec)
```

## 注意
如果一个查询中包含分组、排序和分页，那么它们之间必须按照<span style="color:red">分组->排序->分页</span>的先后顺序排列。