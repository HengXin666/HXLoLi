# 补充: MySsql数据库操作类
## 1. 大自然的馈赠: SqlSession.h
```C++
/**
 * 定义一个数据库操作工具类
 * 参考链接：https://dev.mysql.com/doc/connector-cpp/1.1/en/connector-cpp-apps.html
 */
class SqlSession final
{
private:
    // 数据库连接对象
    Connection* conn;
    // PreparedStatement对象
    PreparedStatement* pstmt;
    // Statement对象
    Statement* stmt;
    // 结果集对象
    ResultSet* res;
    // 释放PreparedStatement对象
    void releasePreparedStatement();
    // 释放Statement对象
    void releaseStatement();
    // 释放结果集对象
    void releaseResultSet();
    // 执行数据更新操作
    int update(const string& sql, const char* fmt, va_list args);
public:
    SqlSession();
    ~SqlSession();

    //************************************
    // Method:    execute
    // FullName:  SqlSession::execute
    // Access:    public 
    // Returns:   bool
    // Description: 执行SQL语句，一般用于DDL
    // Parameter: const string & sql SQL语句
    //************************************
    bool execute(const string& sql);
    
    //************************************
    // Method:    executeUpdate
    // FullName:  SqlSession::executeUpdate
    // Access:    public 
    // Returns:   int 影响数据行数
    // Description: 更新数据，一般用于DML
    // Parameter: const string & sql sql语句
    // Parameter: const char* fmt 可变参数格式字符串,如:%s%i%bi%b%d%dt%n
    // Parameter: ... 可变参数
    //************************************
    int executeUpdate(const string& sql, const char* fmt, ...);

    //************************************
    // Method:    executeUpdate
    // FullName:  SqlSession::executeUpdate
    // Access:    public 
    // Returns:   int 影响数据行数
    // Description: 更新数据，一般用于DML
    // Parameter: const string & sql sql语句
    //************************************
    int executeUpdate(const string& sql);

    //************************************
    // Method:    executeUpdate
    // FullName:  SqlSession::executeUpdate
    // Access:    public 
    // Returns:   int 影响数据行数
    // Description: 更新数据，一般用DML
    // Parameter: const string & sql sql语句
    // Parameter: const SqlParams & params SQL语句填充参数
    //************************************
    int executeUpdate(const string& sql, const SqlParams& params);

    //************************************
    // Method:    executeInsert
    // FullName:  SqlSession::executeInsert
    // Access:    public 
    // Returns:   uint64_t 自动增长ID值
    // Description: 插入数据，用于数据库主键自增的数据表，主键非自增请使用executeUpdate
    // Parameter: const string & sql sql语句
    // Parameter: const char* fmt 可变参数格式字符串,如:%s%i%bi%b%d%dt%n
    // Parameter: ... 可变参数
    //************************************
    uint64_t executeInsert(const string& sql, const char* fmt, ...);

    //************************************
    // Method:    executeInsert
    // FullName:  SqlSession::executeInsert
    // Access:    public 
    // Returns:   uint64_t 自动增长ID值
    // Description: 插入数据，用于数据库主键自增的数据表，主键非自增请使用executeUpdate
    // Parameter: const string & sql sql语句
    //************************************
    uint64_t executeInsert(const string& sql);

    //************************************
    // Method:    executeInsert
    // FullName:  SqlSession::executeInsert
    // Access:    public 
    // Returns:   uint64_t 自动增长ID值
    // Description: 插入数据，用于数据库主键自增的数据表，主键非自增请使用executeUpdate
    // Parameter: const string & sql sql语句
    // Parameter: const SqlParams & params SQL语句填充参数
    //************************************
    uint64_t executeInsert(const string& sql, const SqlParams& params);
    
    //************************************
    // Method:    executeQueryNumerical
    // FullName:  SqlSession::executeQueryNumerical
    // Access:    public 
    // Returns:   uint64_t 整型数字
    // Description: 查询一个整型数字，比如查询count、max这类数据
    // Parameter: const string & sql SQL语句
    // Parameter: const char* fmt 可变参数格式字符串,如:%s%i%bi%b%d%dt%n
    // Parameter: ... 可变参数
    //************************************
    uint64_t executeQueryNumerical(const string& sql, const char* fmt, ...);

    //************************************
    // Method:    executeQueryNumerical
    // FullName:  SqlSession::executeQueryNumerical
    // Access:    public 
    // Returns:   uint64_t 整型数字
    // Description: 查询一个整型数字，比如查询count、max这类数据
    // Parameter: const string & sql SQL语句
    //************************************
    uint64_t executeQueryNumerical(const string& sql);

    //************************************
    // Method:    executeQueryNumerical
    // FullName:  SqlSession::executeQueryNumerical
    // Access:    public 
    // Returns:   uint64_t 整型数字
    // Description: 查询一个整型数字，比如查询count、max这类数据
    // Parameter: const string & sql SQL语句
    // Parameter: const SqlParams & params SQL语句填充参数
    //************************************
    uint64_t executeQueryNumerical(const string& sql, const SqlParams& params);

    //************************************
    // Method:    executeQuery
    // FullName:  SqlSession::executeQuery
    // Access:    public 
    // Returns:   查询结果集合
    // Description: 执行查询
    // Parameter: const string & sql SQL语句
    // Parameter: const M& mapper 查询结果匹配处理
    // Parameter: const char* fmt 可变参数格式字符串,如:%s%i%bi%b%d%dt%n
    // Parameter: ... 可变参数
    //************************************
    template<class T, typename M = Mapper<T>>
    std::list<T> executeQuery(const string& sql, const M& mapper, const char* fmt, ...) {
        std::list<T> list;
        try
        {
            NULL_PTR_CHECK(conn, "connection is null");
            //1 获取prepareStatement对象
            pstmt = conn->prepareStatement(sql);
            //2 处理参数
            SQL_ARG_EXEC_2(pstmt, fmt);
            //3 执行查询
            res = pstmt->executeQuery();
            //4 处理查询结果
            while (res->next()) {
                list.push_back(mapper.mapper(res));
            }
            //5 释放资源
            releaseResultSet();
            releasePreparedStatement();
        }
        catch (const std::exception& e)
        {
            //5 释放资源
            releaseResultSet();
            releasePreparedStatement();
            cerr << "ExecuteQuery Exception. " << e.what() << endl;
        }
        return list;
    }

    //************************************
    // Method:    executeQuery
    // FullName:  SqlSession::executeQuery
    // Access:    public 
    // Returns:   查询结果集合
    // Description: 执行查询
    // Parameter: const string & sql SQL语句
    // Parameter: const M& mapper 查询结果匹配处理
    //************************************
    template<class T, typename M = Mapper<T>>
    std::list<T> executeQuery(const string& sql, const M& mapper) {
        std::list<T> list;
        TryFinally(
            [&] {
                NULL_PTR_CHECK(conn, "connection is null");
                //1 获取prepareStatement对象
                pstmt = conn->prepareStatement(sql);
                //2 执行查询
                res = pstmt->executeQuery();
                //3 处理查询结果
                while (res->next()) {
                    list.push_back(mapper.mapper(res));
                }
            },
            [](const std::exception& e){
                cerr << "ExecuteQuery Exception. " << e.what() << endl;
            },
            [=] {
                //4 释放资源
                releaseResultSet();
                releasePreparedStatement();
            }
        );
        return list;
    }

    //************************************
    // Method:    executeQuery
    // FullName:  SqlSession::executeQuery
    // Access:    public 
    // Returns:   查询结果集合
    // Description: 执行查询
    // Parameter: const string & sql SQL语句
    // Parameter: const M& mapper 查询结果匹配处理
    // Parameter: const SqlParams & params SQL语句填充参数
    //************************************
    template<class T, typename M = Mapper<T>>
    std::list<T> executeQuery(const string& sql, const M& mapper, const SqlParams& params) {
        std::list<T> list;
        try
        {
            NULL_PTR_CHECK(conn, "connection is null");
            //1 获取prepareStatement对象
            pstmt = conn->prepareStatement(sql);
            //2 处理参数
            SQL_ARG_EXEC_3(params, pstmt);
            //3 执行查询
            res = pstmt->executeQuery();
            //4 处理查询结果
            while (res->next()) {
                list.push_back(mapper.mapper(res));
            }
            //5 释放资源
            releaseResultSet();
            releasePreparedStatement();
        }
        catch (const std::exception& e)
        {
            //5 释放资源
            releaseResultSet();
            releasePreparedStatement();
            cerr << "ExecuteQuery Exception. " << e.what() << endl;
        }
        return list;
    }

    // 设置连接编码
    bool setCharset(const std::string& charset);

    // 启动事务
    void beginTransaction();

    // 提交事务
    void commitTransaction();

    // 事务回滚
    void rollbackTransaction();

    // 获取连接对象
    Connection* getConnection() { return this->conn; }
};
```

## 1.1 示例: 使用事务

```C++
bool RepairorderService::removeData(const DeleteMultipleRepairersDTO::Wrapper& idList) {
    RepairorderDAO dao;
    // 创建一个会话
    auto sqlSession = dao.getSqlSession();
    // 开启事务
    sqlSession->beginTransaction();
    bool flag = true;
    for (auto& it : *idList->repairIdList) {
        std::uint64_t id(it);

        if (!dao.deleteById(id)) {
            // 删除失败，回滚
            sqlSession->rollbackTransaction();
            flag = false;
            break;
        }
    }
    // 提交事务
    sqlSession->commitTransaction();
    return flag;
}

// 其中
int RepairorderDAO::deleteById(uint64_t id) {
    string sql = "DELETE FROM `dv_repair` WHERE `repair_id`=?";
    return sqlSession->executeUpdate(sql, "%ull", id);
}
```
