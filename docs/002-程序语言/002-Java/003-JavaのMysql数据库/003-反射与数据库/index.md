# 反射与数据库
数据库查询出的每一条数据基本上都会封装为一个对象，数据库中的每一个字段值都会存储在对象相应的属性中。如果查询结果的每一个字段都与对象中的属性名保持一致，那么就可以使用反射来完成万能查询。

`JdbcUtil`构建演示:

```java
package com.HX.text;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.sql.*;
import java.util.ArrayList;
import java.util.List;

class StuBySql {
    protected int id;
    protected String name;
    protected int sex;
    protected String birthday;
    protected String className;

    public int getId() {
        return id;
    }

    public void setId(int id) {
        this.id = id;
    }

    public String getName() {
        return name;
    }

    public void setName(String name) {
        this.name = name;
    }

    public int getSex() {
        return sex;
    }

    public void setSex(int sex) {
        this.sex = sex;
    }

    public String getBirthday() {
        return birthday;
    }

    public void setBirthday(String birthday) {
        this.birthday = birthday;
    }

    public String getClassName() {
        return className;
    }

    public void setClassName(String className) {
        this.className = className;
    }

    @Override
    public String toString() {
        return "StuBySql{" +
                "id=" + id +
                ", name='" + name + '\'' +
                ", sex=" + sex +
                ", birthday='" + birthday + '\'' +
                ", className='" + className + '\'' +
                '}';
    }
}
```

```java
public class JdbcUtil {
    private static final String url = "jdbc:mysql://localhost:3306/hx_demo?serverTimezone=Asia/Shanghai";
    private static final String username = "root";
    private static final String password = "root";

    static {
        try {
            Class.forName("com.mysql.cj.jdbc.Driver");
        } catch (ClassNotFoundException ex) {
            System.err.println("加载数据库失败!");
            throw new RuntimeException(ex);
        }
    }

    public static void main(String[] args) {
        // 注意, 可以使用 as 起别名以方便反射对应
        String sql = "select id, name, sex, birthday, class as className from stu where id < 10";
        List<StuBySql> stuBySqls = select(sql, StuBySql.class);
        for (StuBySql stu : stuBySqls) {
            System.out.println(stu.toString());
        }

        sql = "update stu set name = '老登' where id = 1";
        System.out.println("受影响行数:" + update(sql));
    }

    /**
     * 万能执行语句
     * @param sql 对应的执行语句
     * @param params 匹配的`?`的占位
     * @return
     */
    public static int update(String sql, Object...params) {
        int res = 0; // 受影响的行数
        Connection connection = null;
        PreparedStatement ps = null;
        try {
            // 进行数据库连接
            connection = DriverManager.getConnection(url, username, password);
            // 创建SQL预处理器
            ps = buildPrepareStatement(sql, connection, params);
            // 执行语句
            res = ps.executeUpdate();
        } catch (SQLException e) {
            throw new RuntimeException(e);
        } finally {
            close(ps, connection);
        }
        return res;
    }

    /**
     * 通过params与sql的`?`进行匹配, 创建预处理SQL
     * @param sql
     * @param connection
     * @param params
     * @return
     * @throws SQLException
     */
    private static PreparedStatement buildPrepareStatement(String sql, Connection connection, Object...params) throws SQLException {
        PreparedStatement ps = connection.prepareStatement(sql);
        if (params != null) {
            for (int i = 0; i < params.length; ++i) {
                ps.setObject(i + 1, params[i]);
            }
        }
        return ps;
    }

    /**
     * 创建实例
     * @param clazz 目标实例的类型
     * @param rs 查询结果集
     * @return 实例
     * @param <T>
     * @throws InstantiationException
     * @throws IllegalAccessException
     * @throws NoSuchMethodException
     */
    private static<T> T createInstance(Class<T> clazz, ResultSet rs) throws InstantiationException, IllegalAccessException, NoSuchMethodException {
        T t = clazz.newInstance();
        // 获取对应的所有set方法 (通过字段推理出)
        for (Field field : clazz.getDeclaredFields()) {
            String funName = field.getName();
            String setFunName = "set" + funName.substring(0, 1).toUpperCase() + funName.substring(1);
            Method method = clazz.getDeclaredMethod(setFunName, field.getType());

            try { // rs.getObject(类名, 返回时 转变为 field.getType() 类型) 获取对应类名的字段
                method.invoke(t, rs.getObject(funName, field.getType()));
            } catch (Exception e) {

            }
        }

        return t;
    }

    /**
     * 万能查询
     * @param sql 对应的查询语句
     * @param clazz 目标类型
     * @param params 限制(?占位的)
     * @return 查询结果
     * @param <T>
     */
    public static<T> List<T> select(String sql, Class<T> clazz, Object...params) {
        List<T> res = new ArrayList<>();
        Connection connection = null;
        PreparedStatement ps = null;
        ResultSet rs = null;
        try {
            // 进行数据库连接
            connection = DriverManager.getConnection(url, username, password);

            // 创建SQL预处理器
            ps = buildPrepareStatement(sql, connection, params);

            rs = ps.executeQuery();
            while (rs.next()) {
                res.add(createInstance(clazz, rs));
            }
        } catch (SQLException | InstantiationException | IllegalAccessException | NoSuchMethodException e) {
            throw new RuntimeException(e);
        } finally {
            close(rs, ps, connection); // 关闭连接, 注意顺序
        }

        return res;
    }

    /**
     * 关闭连接, 参数在前的先关闭
     * @param closeables
     */
    private static void close(AutoCloseable...closeables) {
        if (closeables != null) {
            for (AutoCloseable closeable : closeables) {
                try {
                    closeable.close();
                } catch (Exception e) {
                    throw new RuntimeException(e);
                }
            }
        }
    }
}
```