# JDBC
## 概念
**JDBC (Java Database Connection)** 是Java数据库连接技术的简称，提供连接数据库的能力。

## JDBC API
Java 作为目前世界上最流行的高级开发语言，当然不可能考虑去实现各种数据库的连接与操作。但 Java 语言的开发者对数据库的连接与操作提供了相关的接口，供各大数据库厂商去实现。这些接口位于`java.sql`包中。

### Driver
`java.sql.Driver`: 数据库厂商提供的`JDBC`驱动包中必须包含该接口的实现，该接口中就包含连接数据库的功能。

```java
// 根据给定的数据库url地址连接数据库
// info 即账号, 密码等信息
Connection connect(String url, java.util.Properties info) throws SQLException;
```

### DriverManager
`java.sql.DriverManager`: 数据库厂商的提供的`JDBC`驱动交给`DriverManager`来管理，`DriverManager`主要负责获取数据库连接对象`Connection`

```java
// 通过给定的账号、密码和数据库地址获取一个连接
public static Connection getConnection(String url, String user, String password) throws SQLException
```
### Connection
`java.sql.Connection`: 连接接口，数据库厂商提供的`JDBC`驱动包中必须包含该接口的实现，该接口主要提供与数据库的交互功能

```java
// 创建一个SQL语句执行对象
Statement createStatement() throws SQLException;
// 创建一个预处理SQL语句执行对象
PreparedStatement prepareStatement(String sql) throws SQLException;
// 创建一个存储过程SQL语句执行对象
CallableStatement prepareCall(String sql) throws SQLException;
// 设置该连接上的所有操作是否执行自动提交
void setAutoCommit(boolean autoCommit) throws SQLException;
// 提交该连接上至上次提交以来所作出的所有更改
void commit() throws SQLException;
// 回滚事务，数据库回滚到原来的状态
void rollback() throws SQLException;
// 关闭连接
void close() throws SQLException;
// 设置事务隔离级别
void setTransactionIsolation(int level) throws SQLException;
```

```java
// 不支持事务
int TRANSACTION_NONE = 0;
// 读取未提交的数据
int TRANSACTION_READ_UNCOMMITTED = 1;
// 读取已提交的数据
int TRANSACTION_READ_COMMITTED = 2;
// 可重复读
int TRANSACTION_REPEATABLE_READ = 4;
// 串行化
int TRANSACTION_SERIALIZABLE = 8;
```

### Statement
`java.sql.Statement`: SQL语句执行接口，数据库厂商提供的`JDBC`驱动包中必须包含该接口的实现，该接口主要提供执行 SQL 语句的功能

```java
// 执行查询，得到一个结果集
ResultSet executeQuery(String sql) throws SQLException;
// 执行更新，得到受影响的行数
int executeUpdate(String sql) throws SQLException;
// 关闭SQL语句执行器
void close() throws SQLException;
// 将SQL语句添加到批处理执行SQL列表中
void addBatch( String sql ) throws SQLException;
// 执行批处理，返回列表中每一条SQL语句的执行结果(每个元素是受影响的行数)
int[] executeBatch() throws SQLException;
```

### ResultSet
`java.sql.ResultSet`: 查询结果集接口，数据库厂商提供的`JDBC`驱动包中必须包含该接口的实现，该接口主要提供查询结果的获取功能

```java
// 光标从当前位置（默认位置位为0）向前移动一行，如果存在数据，则返回true，否则返回false
boolean next() throws SQLException;
// 关闭结果集
void close() throws SQLException;
// 获取指定列的字符串值
String getString(int columnIndex) throws SQLException;
// 获取指定列的布尔值
boolean getBoolean(int columnIndex) throws SQLException;
// 获取指定列的整数值
int getInt(Sting columnName) throws SQLException;
// 获取指定列的对象
Object getObject(int columnIndex, Class type) throws SQLException;
// 获取结果集元数据：查询结果的列名称、列数量、列别名等等
ResultSetMetaData getMetaData() throws SQLException;
// 光标从当前位置（默认位置位为0）向后移动一行，如果存在数据，则返回true，否则返回false
boolean previous() throws SQLException;
```

<div style="margin-top: 80px;">

---
</div>

## JDCB 操作步骤
### 0. 引入驱动包
新建工程后，将 `mysql-connector-java.jar` 引入工程中

### 1. 加载驱动

```java
//MySQL版本为 5.0 的加载方式
//Class.forName("com.mysql.jdbc.Driver");

//MySQL版本为 8.0 的加载方式
Class.forName("com.mysql.cj.jdbc.Driver");
```

### 2. 获取连接

```java
// jdbc:使用jdbc连接技术
// mysql://localhost:3306 使用的是MySQL数据库协议，访问的是本地计算机3306端口
// hx_demo哪个库
// ? 后面是指定时区(亚洲/上海) mysql8.0要求的(?)
String url = "jdbc:mysql://localhost:3306/hx_demo?serverTimezone=Asia/Shanghai";
String username = "root";
String password = "root";
Connection connection = DriverManager.getConnection(url, username, password);
```

### 3. 创建 SQL 语句执行器

```java
Statement statement = connection.createStatement();
```

### 4. 执行 SQL 语句

```java
// 查询
ResultSet rs = statement.executeQuery(sql);
while(rs.next()) {
    // 示例: 获取列信息 注意, 数字是从1开始的! 对于的是 [select * ...] 的*的顺序
    int id = rs.getInt(1);
    String name = rs.getString(2);
}

// 更新
int affectedRows = statement.executeUpdate();
```

### 5. 释放资源

```java
rs.close();
statement.close();
connection.close();
```

**示例**:

```java
import java.sql.*;

public class Main {
    public static void main(String[] args) {
        //MySQL版本为 8.0 的加载方式
        try {
            // 选择使用的加载方式
            Class.forName("com.mysql.cj.jdbc.Driver");
            // 进行数据库连接
            String url = "jdbc:mysql://localhost:3306/hx_demo?serverTimezone=Asia/Shanghai";
            String username = "root";
            String password = "root";
            Connection connection = DriverManager.getConnection(url, username, password);
            // 创建SQL语句执行器
            Statement statement = connection.createStatement();

            // 执行sql语句
            String sql = "select * from stu";
            ResultSet resultSet = statement.executeQuery(sql);
            while (resultSet.next()) {
                int id = resultSet.getInt(1);
                String name = resultSet.getString(2);
                int sex = resultSet.getInt(3);
                String biday = resultSet.getString(4);
                String className = resultSet.getString(5);
                System.out.println(id + "|" + name + "|" + sex + "|" + biday + "|" + className);
            }

            sql = "update stu set name = '老登' where id = 1";

            System.out.println("更新了: " + statement.executeUpdate(sql) + "行");

            // 关闭资源
            resultSet.close();
            statement.close();
            connection.close();
        } catch (ClassNotFoundException | SQLException e) {
            throw new RuntimeException(e);
        }
    }
}
```

## 预处理SQL
在日常开发中，我们经常会根据用户输入的信息从数据库中进行数据筛选，现有 stu 表数据如下:

```sql
+----+----------+------+------------+---------+
| id | name     | sex  | birthday   | class   |
+----+----------+------+------------+---------+
|  1 | 老登     |    2 | 1997-07-18 | 计科2班 |
|  2 | 宋宛丝   |    0 | 1992-05-24 | 软工1班 |
|  3 | 卫鸿文   |    0 | 1990-06-15 | 计科2班 |
|  4 | 袁巧蕊   |    0 | 1993-02-09 | 计科2班 |
|  5 | 罗浩轩   |    0 | 1991-03-13 | 计科1班 |
|  6 | 羊海瑶   |    1 | 1992-10-04 | 计科2班 |
|  7 | 伏鸿远   |    2 | 1993-10-12 | 计科1班 |
|  8 | 赫连春蕾 |    2 | 1993-03-15 | 计科1班 |
|  9 | 柯浩阔   |    0 | 1999-02-24 | 软工2班 |
+----+----------+------+------------+---------+
```

现要根据用户输入的学生姓名查询学生信息。

```java
Scanner sc = new Scanner(System.in);
System.out.println("请输入学生姓名：");
String name = sc.next();
String sql = "SELECT id, name, sex, age FROM stu WHERE name='" + name + "'";
```

如果此时用户输入信息为: `张华' or 1='1`，那么，上面的代码执行后 SQL 语句变为

```sql
SELECT id, name, sex, age FROM stu WHERE name='张华' or 1='1'
```

因为 `1='1'` 是恒成立的, 会被套出所有的数据~

显然查询的结果发生了变化，这样的情况被称作为 <b style="color:red">SQL 注入</b>。

为了防止 SQL 注入，Java 提供了`PreparedStatement`接口对 SQL 进行预处理，该接口是`Statement`接口的子接口，其常用方法如下:

```java
// 执行查询，得到一个结果集
ResultSet executeQuery() throws SQLException;
// 执行更新，得到受影响的行数
int executeUpdate() throws SQLException;
// 使用给定的整数值设置给定位置的参数
void setInt(int parameterIndex, int x) throws SQLException;
// 使用给定的长整数值设置给定位置的参数
void setLong(int parameterIndex, long x) throws SQLException;
// 使用给定的双精度浮点数值设置给定位置的参数
void setDouble(int parameterIndex, double x) throws SQLException;
// 使用给定的字符串值设置给定位置的参数
void setString(int parameterIndex, String x) throws SQLException;
// 使用给定的对象设置给定位置的参数
void setObject(int parameterIndex, Object x) throws SQLException;
// 获取结果集元数据
ResultSetMetaData getMetaData() throws SQLException;
```

如何获取 `PreparedStatement` 接口对象呢?

```java
PreparedStatement ps = connection.prepareStatement(sql);
```

`PreparedStatement`是如何进行预处理的？

使用`PreparedStatement`时，SQL 语句中的参数一律使用`?`号来进行占位，然后通过调用`setXxx()`方法来对占位的`?`号进行替换。从而将参数作为一个整体进行查询。

上面的示例使用 `PreparedStatement` 编写 SQL 语句为:

```java
Scanner sc = new Scanner(System.in);
System.out.println("请输入学生姓名：");
String name = sc.next();
String sql = "SELECT id, name, sex, age FROM stu WHERE name=?";
PreparedStatement ps = connection.prepareStatement(sql);
ps.setString(1, name);
ResultSet rs = ps.executeQuery();
```

> 建议都使用预处理SQL, 因为它 "安全"

完整代码:

```java
public class Main {
    static Scanner sc = new Scanner(System.in);

    public static void main(String[] args) {
        // 选择使用的加载方式
        try {
            Class.forName("com.mysql.cj.jdbc.Driver");
            // 进行数据库连接
            String url = "jdbc:mysql://localhost:3306/hx_demo?serverTimezone=Asia/Shanghai";
            String username = "root";
            String password = "root";
            Connection connection = DriverManager.getConnection(url, username, password);

            String _name;
            System.out.println("请输入需要查询的名称: ");
            _name = sc.nextLine();

            // 创建SQL预处理器
            String sql = "select * from stu where name = ?";
            PreparedStatement ps = connection.prepareStatement(sql);
            ps.setString(1, _name);

            ResultSet rs = ps.executeQuery();
            while (rs.next()) {
                int id = rs.getInt(1);
                String name = rs.getString(2);
                int sex = rs.getInt(3);
                String biday = rs.getString(4);
                String className = rs.getString(5);
                System.out.println(id + "|" + name + "|" + sex + "|" + biday + "|" + className);
            }

        } catch (ClassNotFoundException | SQLException e) {
            throw new RuntimeException(e);
        }
    }
}
```
