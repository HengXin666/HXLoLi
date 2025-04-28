# 存储过程、函数、触发器和视图
## 第一节 变量
在MySQL中，变量分为四种类型，即局部变量、用户变量、会话变量和全局变量。其中局部变量和用户变量在实际应用中使用较多，会话变量和全局变量使用较少，因此作为了解即可。

### 1. 全局变量
MySQL全局变量会影响服务器整体操作，当服务启动时，它将所有全局变量初始化为默认值。要想更改全局变量，必须具有管理员权限。其作用域为服务器的整个生命周期。

```sql
-- 显示所有的全局变量
SHOW GLOBAL VARIABLES;

-- 设置全局变量的值的两种方式
SET GLOBAL sql_warnings = ON; -- GLOBAL不能省略
SET @@GLOBAL.sql_warnings = OFF;

-- 查询全局变量的值的两种方式
SELECT @@GLOBAL.sql_warnings;
SHOW GLOBAL VARIABLES LIKE '%sql_warnings%';
```

### 2. 会话变量
MySQL会话变量是服务器为每个连接的客户端维护的一系列变量。其作用域仅限于当前连接，因此，会话变量是独立的。

```sql
-- 显示所有的会话变量
SHOW SESSION VARIABLES;

-- 设置会话变量的值的三种方式
SET SESSION auto_increment_increment = 1;
SET @@SESSION.auto_increment_increment = 2;

-- 当省略SESSION关键字时，默认缺省为SESSION，即设置会话变量的值
SET auto_increment_increment = 3;

-- 查询会话变量的值的三种方式
SELECT @@auto_increment_increment;
SELECT @@SESSION.auto_increment_increment;

-- SESSION关键字可省略,也可用关键字LOCAL替代
SHOW SESSION VARIABLES LIKE '%auto_increment_increment%';
SET @@LOCAL.auto_increment_increment = 1;
SELECT @@LOCAL.auto_increment_increment;
```

### 3. 用户变量
MySQL用户变量，MySQL中用户变量不用提前申明，在用的时候直接用“@变量名”使用就可以了。其作用域为当前连接。

```sql
-- 第一种用法，使用SET时可以用“=”或“:=”两种赋值符号赋值
SET @age = 19;

-- 第二种用法，使用SELECT时必须用“:=”赋值符号赋值
SELECT @age := 22;
SELECT @age := age FROM stu WHERE `name` = '张华';

-- 第三种用法，使用SELECT... INTO语句赋值
SELECT age INTO @age FROM stu WHERE `name` = '张华';
SELECT @age;
```

### 4. 局部变量
MySQL局部变量，只能用在`BEGIN`/`END`语句块中，比如存储过程中的`BEGIN`/`END`语句块。

```sql
-- 定义局部变量
DECLARE age INT(3) DEFAULT 0;

-- 为局部变量赋值
SET age = 10;
SELECT age := 10;
SELECT 10 INTO age;
SELECT age;
```
<div style="margin-top: 80px;">

---
</div>

## 第二节 存储过程
### 1. 概念
在大型数据库系统中，存储过程是一组为了完成特定功能而存储在数据库中的SQL语句集，一次编译后永久有效
### 2. 为什么要使用存储过程
- 运行速度快：

    在存储过程创建的时候，数据库已经对其进行了一次解析和优化。存储过程一旦执行，在内存中就会保留一份这个存储过程，下次再执行同样的存储过程时，可以从内存中直接调用，所以执行速度会比普通SQL快。 
- 减少网络传输：

    存储过程直接就在数据库服务器上跑，所有的数据访问都在数据库服务器内部进行，不需要传输数据到其它服务器，所以会减少一定的网络传输。
- 增强安全性：

    提高代码安全，防止 SQL被截获、篡改。

### 3. 如何使用存储过程
语法:

```sql
-- 声明分隔符
[DELIMITER $$]
CREATE PROCEDURE 存储过程名称 ([IN | OUT | INOUT] 参数名1 数据类型, [[IN | OUT | INOUT] 参数名2 数据类型, . , [IN | OUT | INOUT] 参数名n 数据类型])

-- 语句块开始
BEGIN
    -- SQL语句集
END[$$]

-- 还原分隔符
[DELIMITER ; ]

-- 调用存储过程
CALL 存储过程名(参数1,参数2,…);
```

示例:

```sql
-- 使用存储过程完成银行转账业务
DELIMITER aa
CREATE PROCEDURE transfer(IN id_1 BIGINT, IN id_2 BIGINT, IN money DOUBLE(20, 3))
BEGIN
    UPDATE account SET balance = balance - money WHERE account = id_1;
    UPDATE account SET balance = balance + money WHERE account = id_2;
END aa
DELIMITER ;

-- 如果转账账户余额不足，上面的SQL代码依然可以正常执行，只是执行完后，转账账户的余额变为了负数。这显然不符合常理。因此需要修正。
DROP PROCEDURE IF EXISTS transfer;
CREATE PROCEDURE transfer(IN id_1 BIGINT, IN id_2 BIGINT, IN money DOUBLE(20, 3))
BEGIN
    -- 定义变量表示执行结果：0-失败，1-成功
    DECLARE result TINYINT(1) DEFAULT 0;

    UPDATE account SET balance = balance - money WHERE account = id_1 AND balance >= money;
    IF ROW_COUNT() = 1 THEN
        UPDATE account SET balance = balance + money WHERE account = id_2;
        IF ROW_COUNT() = 1 THEN
            SET result = 1;
        END IF;
    END IF;
    
    SELECT result;
END

CALL transfer(123456, 123457, 2000);
```
如果转账账户已经将钱转出去，而在执行目标账户增加余额的时候出现了异常或者目标账户输入错误，此时应该怎么办呢？

MySQL对数据的操作提供了事务的支持，用来保证数据的一致性,可以有效的解决此类问题。

### 4. 事务
#### 4.1 什么是事务
**事务(Transaction)** 是访问并可能操作各种数据项的一个数据库操作序列，这些操作要么全部执行,要么全部不执行，是一个不可分割的工作单位。事务由事务开始与事务结束之间执行的全部数据库操作组成。

#### 4.2 事务的特性(ACID)
- 原子性（Atomicity）

    - 事务的各元素是不可分的（原子的）,它们是一个整体。要么都执行，要么都不执行。

- 一致性（Consistency）

    - 当事务完成时，必须保证所有数据保持一致状态。当转账操作完成时，所有账户的总金额应该保持不变，此时数据处于一致性状态；如果总金额发生了改变，说明数据处于非一致性状态。

- 隔离性（Isolation）

    - 对数据操作的多个并发事务彼此独立，互不影响。比如张三和李四同时都在进行转账操作，但彼此都不影响对方。

- 持久性（Durability）

    - 对于已提交事务，系统必须保证该事务对数据库的改变不被丢失，即使数据库出现故障
 
#### 4.3 事务解决银行转账问题

```sql
-- 使用事务
DROP PROCEDURE IF EXISTS transfer;
CREATE PROCEDURE transfer(IN id_1 BIGINT, IN id_2 BIGINT, IN money DOUBLE(20, 3))
BEGIN
    -- 定义变量表示执行结果：0-失败，1-成功
    DECLARE result TINYINT(1) DEFAULT 1; -- 默认为 1
    
    -- 声明SQLEXCEPTION处理器，当有SQLEXCEPTION发生时，错误标识符的值设为0
    -- 发生SQLEXCEPTION时的处理方式：CONTINUE，EXIT
        -- CONTINUE表示即使有异常发生，也会执行后面的语句
        -- EXIT表示，有异常发生时，直接退出当前存储过程
    DECLARE CONTINUE HANDLER FOR SQLEXCEPTION SET result = 0;

    -- 开启事务
    START TRANSACTION;
    
    UPDATE account SET balance = balance - money WHERE account = id_1 AND balance >= money;
    -- 设置result的值为SQL执行后受影响的行数
    SET result = ROW_COUNT();
    
    IF result = 1 THEN
        UPDATE account SET balance = balance + money WHERE account = id_2;
        SET result = ROW_COUNT();
    END IF;
    
    -- 如果result的值为1，表示所有操作都成功，提交事务
    IF result = 1 THEN COMMIT;
    -- 否则，表示操作存在失败的情况，事务回滚，数据恢复到更改之前的状态
    ELSE ROLLBACK;
    END IF;
    
    SELECT result;
END
```

#### 4.5 存储过程输出
```sql
-- 使用事务并且有传出参数
DROP PROCEDURE IF EXISTS transfer;
CREATE PROCEDURE transfer(IN id_1 BIGINT, IN id_2 BIGINT, IN money DOUBLE(20, 3), OUT result TINYINT(1))
BEGIN
    -- 声明SQLEXCEPTION处理器，当有SQLEXCEPTION发生时，错误标识符的值设为0
    -- 发生SQLEXCEPTION时的处理方式：CONTINUE，EXIT
        -- CONTINUE表示即使有异常发生，也会执行后面的语句
        -- EXIT表示，有异常发生时，直接退出当前存储过程
    DECLARE CONTINUE HANDLER FOR SQLEXCEPTION SET result = 0;

    -- 开启事务
    START TRANSACTION;
    
    UPDATE account SET balance = balance - money WHERE account = id_1 AND balance >= money;
    -- 设置result的值为SQL执行后受影响的行数
    SET result = ROW_COUNT();
    
    IF result = 1 THEN
        UPDATE account SET balance = balance + money WHERE account = id_2;
        SET result = ROW_COUNT();
    END IF;
    
    -- 如果result的值为1，表示所有操作都成功，提交事务
    IF result = 1 THEN COMMIT;
    -- 否则，表示操作存在失败的情况，事务回滚，数据恢复到更改之前的状态
    ELSE ROLLBACK;
    END IF;
END

-- 使用/查询
CALL transfer(123457, 123456, 8000, @re); -- @re 是用户变量
SELECT @re;
```

<div style="margin-top: 80px;">

---
</div>

## 第三节 自定义函数
### 1. 概念
函数就是在大型数据库系统中，一组为了完成特定功能而存储在数据库中的SQL 语句集，一次编译后永久有效

### 2. 自定义函数
MySQL本身提供了一些内置函数，这些函数给我们日常的开发和数据操作带来了很大的便利，比如聚合函数`SUM()`、`AVG()`以及日期时间函数等。但这并不能完全满足开发的需要，有时我们需要一个函数来完成一些复杂功能的实现，而MySQL中又没有这样的函数，因此，我们需要自定义函数来实现。

### 3. 如何使用自定义函数
语法

```sql
CREATE FUNCTION 函数名称 (参数名1 数据类型, 参数名2 数据类型, . , 参数名n 数据类型])
RETURNS 数据类型
-- 函数特征：
-- DETERMINISTIC: 不确定的
-- NO SQL：没有SQL语句，当然也不会修改数据
-- READS SQL DATA： 只是读取数据，不会修改数据
-- MODIFIES SQL DATA：要修改数据
-- CONTAINS SQL：包含了SQL语句
DETERMINISTIC | NO SQL | READS SQL DATA | MODIFIES SQL DATA | CONTAINS SQL
-- 语句块开始
BEGIN
    -- SQL语句集
RETURN 结果;
-- 语句块结束
END
```

示例：使用函数实现求score表中的成绩最大差值

```sql
CREATE FUNCTION getCaZhi()
RETURNS DOUBLE(5, 2)
READS SQL DATA
BEGIN
    RETURN (SELECT MAX(score) - MIN(score) FROM score);
END

-- 调用 函数
SELECT getCaZhi();
```

### 4. 循环结构

```sql
-- 类似于 while 循环
WHILE 循环条件 DO
    -- SQL语句集
END WHILE；

-- 类似于 do while 循环
REPEAT
    -- SQL语句集
UNTIL 循环终止条件 END REPEAT;

-- 类似于 for 循环
标号: LOOP
    -- SQL语句集
    IF 循环终止条件 THEN LEAVE 标号;
    END IF;
END LOOP;
```

示例: 使用函数实现求0~给定的任意整数的累加和

```sql
-- 方法一
CREATE FUNCTION getSum(n INT)
RETURNS INT
NO SQL
BEGIN
    DECLARE num INT DEFAULT 1;
    DECLARE sum INT DEFAULT 0;
    
    WHILE num <= n DO
        SET sum := sum + num;
        SET num := num + 1;
    END WHILE;
    RETURN sum;
END

SELECT getSum(100);

-- 方法二
DROP FUNCTION IF EXISTS getSum2;
CREATE FUNCTION getSum2(n INT) RETURNS INT
NO SQL
BEGIN
    DECLARE num INT DEFAULT 0;
    DECLARE sum INT DEFAULT 0;
    REPEAT
        SET sum := sum + num;
        SET num := num + 1;
    UNTIL num > n END REPEAT; -- 注意这个是退出条件, 而不是继续循环的条件
    RETURN sum;
END

SELECT getSum2(100);

-- 方法三
DROP FUNCTION IF EXISTS getSum3;
CREATE FUNCTION getSum3(n INT) RETURNS INT
NO SQL
BEGIN
    DECLARE num INT DEFAULT 0;
    DECLARE sum INT DEFAULT 0;
    a: LOOP
        SET sum := sum + num;
        SET num := num + 1;
        IF num > n THEN LEAVE a;
        END IF;
    END LOOP;
    RETURN sum;
END

SELECT getSum3(100);
```

练习:

```sql
-- 练习: 使用函数实现生成一个指定长度的随机字符串
DROP FUNCTION IF EXISTS getRandStr;
CREATE FUNCTION getRandStr(len INT) RETURNS CHAR(255)
NO SQL
BEGIN
    DECLARE i INT DEFAULT 0;
    DECLARE res CHAR(255) DEFAULT '';
    
    WHILE i < len DO
    SET i := i + 1;
    SET res := CONCAT(res, SUBSTR('abcdefghijklmnopquvwxyzABCDEFGHIJKLMNOPQUVWXYZ0123456789', 57 * RAND(), 1));
    END WHILE;
    RETURN res;
END

SELECT getRandStr(6);
```

<div style="margin-top: 80px;">

---
</div>

## 第四节 触发器
### 1. 概念
**触发器(trigger)** 是用来保证数据完整性的一种方法，由事件来触发，比如当对一个表进行`增删改`操作时就会被激活执行。经常用于加强数据的完整性约束和业务规则

### 2. 如何定义触发器

```sql
DROP TRIGGER [IF EXISTS] 触发器名称;
-- 创建触发器
-- 触发时机为BEFORE(之前)或者AFTER(之后)
-- 触发事件，为INSERT 、 UPDATE或者DELETE
CREATE TRIGGER 触发器名称 {BEFORE|AFTER} {INSERT|UPDATE|DELETE} ON 表名 FOR EACH ROW -- {}代表选择其一
BEGIN
    -- 执行的SQL操作
END
```

### 3. 触发器类型
- BEFORE 触发器

    - 应用场景：当您需要在数据更新到数据库之前进行校验、修改数据或实施某些业务规则时，使用 BEFORE 触发器。例如，您可以在插入记录之前验证数据的有效性，或在更新记录之前自动设置某些字段的值。

    - 特点：BEFORE 触发器允许您阻止无效的数据被写入数据库，因为您可以在触发器内部使用信号（SIGNAL SQLSTATE '45000'）来抛出错误，从而阻止事务继续。

- AFTER 触发器

    - 应用场景：当您需要在数据已经成功更新到数据库之后执行一些操作，如同步更新其他表中的数据、记录日志或执行某些后续处理时，使用 AFTER 触发器。例如，您可以在一条记录被插入后自动向日志表中插入一条日志记录。

    - 特点：AFTER 触发器不能直接阻止事务的执行，因为它们在数据已经被修改后触发。但它们非常适合于那些不需要改变即将存储的数据，仅需对数据库操作做出响应的场景。


|触发器类型|NEW和OLD的使用|
|:-|:-|
|INSERT触发器|NEW表示将要或者已经新增的数据|
|UPDATE触发器|OLD表示将要或者已经修改的数据，NEW表示将要修改的数据|
|DELETE触发器|OLD表示将要或者已经删除的数据|

#### 4. 触发器使用场景
场景一

现有商品表goods和订单表order，每一个订单的生成都意味着商品数量的减少，请使用触发器完成这一过程。

```sql
DROP TRIGGER IF EXISTS addOrder;
CREATE TRIGGER addOrder AFTER INSERT ON `order` FOR EACH ROW
BEGIN
    UPDATE goods SET number = number - NEW.sale_count WHERE id = NEW.goods_id;
END

-- 测试代码
INSERT INTO `order` (goods_id, sales_id, sale_count, created_time, state) VALUES (1, 1, 6, '2024-2-9', 1);
```

场景二

现有商品表goods和订单order，每一个订单的取消都意味着商品数量的增加，请使用触发器完成这一过程。


```sql
CREATE TRIGGER delOrder AFTER DELETE ON `order` FOR EACH ROW
BEGIN
    UPDATE goods SET number = number + OLD.sale_count WHERE id = OLD.goods_id;
END

-- 测试代码
DELETE FROM `order` WHERE id = 350002;
```

场景三

现有商品表goods和订单表order，每一个订单购买数量的更新都意味着商品数量的变动，请使用触发器完成这一过程。


```sql
CREATE TRIGGER upOrder AFTER UPDATE ON `order` FOR EACH ROW
BEGIN
    UPDATE goods SET number = number - (NEW.sale_count - OLD.sale_count) WHERE id = OLD.goods_id;
END

-- 测试代码
UPDATE `order` SET sale_count = 1 WHERE id = 350003;
```

<div style="margin-top: 80px;">

---
</div>

## 第五节 视图
### 1. 概念
视图是一张虚拟表，本身并不存储数据，当SQL操作视图时所有数据都是从其他表中查出来

### 2. 如何使用视图

```sql
-- 创建视图
CREATE VIEW 视图名称 AS SELECT 列1[,列2,…] FROM 表名 WHERE 条件;
-- 更新视图
CREATE OR REPLACE VIEW 视图名称 AS SELECT 列1[,列2,…] FROM 表名 WHERE 条件;
-- 删除视图
DROP VIEW IF EXISTS 视图名称;
```

### 3. 为什么使用视图
定制用户数据，聚焦特定的数据。例如：如果频繁获取销售人员编号、姓名和代理商名称，可以创建视图

```sql
-- 视图
CREATE OR REPLACE VIEW salesInfo AS 
SELECT
    a.id,
    a.`name` saleName,
    b.`name` agentName 
FROM
    sales a,
    agent b 
WHERE
    a.agent_id = b.id;

SELECT * FROM salesInfo
```

简化数据操作。例如：进行关联查询时，涉及到的表可能会很多，这时写的SQL语句可能会很长，如果这个动作频繁发生的话，可以创建视图


```sql
DROP VIEW IF EXISTS searchOrderDetail;
CREATE OR REPLACE VIEW searchOrderDetail AS SELECT
    a.id regionId,
    a.`name` regionName,
    b.id agentId,
    b.`name` agentName,
    c.id saleId,
    c.`name` saleName,
    d.sale_count saleCount,
    d.created_time createdTime,
    e.`name` goodsName 
FROM
    region a,
    agent b,
    sales c,
    `order` d,
    goods e 
WHERE
    a.id = b.region_id 
    AND b.id = c.agent_id 
    AND c.id = d.sales_id 
    AND d.goods_id = e.id;
    
SELECT * FROM searchOrderDetail;
```

提高安全性能。例如：用户密码属于隐私数据，用户不能直接查看密码。可以使用视图过滤掉这一字段


```sql
DROP VIEW IF EXISTS userInfo;
CREATE OR REPLACE VIEW userInfo AS
SELECT
    username,
    salt,
    failure_times,
    last_log_time
FROM
    `user`;

SELECT username, salt FROM userInfo;
```

注意：<span style="color:red">视图并不能提升查询速度，只是方便了业务开发，但同时也加大了数据库服务器的压力，因此，需要合理的使用视图</span>