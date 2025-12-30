# sqlpp11

一个知名的 仅C++11 的类型安全的SQL侵入式库 (https://github.com/rbock/sqlpp11)

## 一、使用效果展示

看看它如何使用:

```sql
-- 我们有如下表
CREATE TABLE foo (
    id bigint,
    name varchar(50),
    hasFun bool
);
```

```cpp
// 假设你定义好了 TabFoo
TabFoo foo;
Db db(/* some arguments*/);

// selecting zero or more results, iterating over the results
for (const auto& row : db(select(foo.name, foo.hasFun).from(foo).where(foo.id > 17 and foo.name.like("%bar%")))) {
    if (row.name.is_null())
        std::cerr << "name is null, will convert to empty string" << std::endl;
    std::string name = row.name;   // string-like fields are implicitly convertible to string
    bool hasFun = row.hasFun;      // bool fields are implicitly convertible to bool
}

// selecting ALL columns of a table
for (const auto& row : db(select(all_of(foo)).from(foo).where(foo.hasFun or foo.name == "joker"))) {
    int64_t id = row.id; // numeric fields are implicitly convertible to numeric c++ types
}

// selecting zero or one row, showing off with an alias:
SQLPP_ALIAS_PROVIDER(cheese);
if (const auto& row = db(select(foo.name.as(cheese)).from(foo).where(foo.id == 17))) {
    std::cerr << "found: " << row.cheese << std::endl;
}

// selecting a single row with a single result:
return db(select(count(foo.id)).from(foo).unconditionally()).front().count;

/* Of course there are joins and subqueries, more functions, order_by, group_by etc.
These will be documented soon. */

// A sample insert
db(insert_into(foo).set(foo.id = 17, foo.name = "bar", foo.hasFun = true));

// A sample update
db(update(foo).set(foo.hasFun = not foo.hasFun).where(foo.name != "nobody"));

// A sample delete
db(remove_from(foo).where(not foo.hasFun));
```

非常的清爽, 完全不用编写任何烦人的、没有检查的 SQL 语句. 纯 C++ 编写, 并且类型安全.

> [!TIP]
> 但是令人不舒服的是, 它实际上需要一种`反射`, 故此处就提供了一个 `py` 脚本, 让用户导出 sql 表结构, 然后让 `py` 生成代码.

## 二、源码解析 (粗略)

我们关注: `tests/sqlite3/usage/Select.cpp`

```cpp
// 节选
db(select(all_of(tab)).from(tab).where((tab.alpha + tab.alpha) > 3));
db(select(all_of(tab)).from(tab).where((tab.beta + tab.beta) == ""));
db(select(all_of(tab)).from(tab).where((tab.beta + tab.beta).like(R"(%'\"%)")));
```

我们主要学习 `(tab.alpha + tab.alpha) > 3` 这种为何合法. 因为实在太妙了! 第一次见到的时候 ~~(我以为是魔法, 自动生成的...)~~

### 2.1 py 一键生成

理论上生成的就是这些:

```cpp
namespace TabSample_ {
struct Alpha {
    struct _alias_t {
        static constexpr const char _literal[] = "alpha"; // 字段名
        using _name_t = sqlpp::make_char_sequence<sizeof(_literal), _literal>;

        template <typename T>
        struct _member_t {
            T alpha;
            T& operator()() { return alpha; }
            const T& operator()() const { return alpha; }
        };
    };
    // 一些表属性, 比如类型是 bigint
    using _traits = ::sqlpp::make_traits<::sqlpp::bigint,
                                         ::sqlpp::tag::must_not_insert,
                                         ::sqlpp::tag::must_not_update,
                                         ::sqlpp::tag::can_be_null>;
};

struct Beta { ... };

struct Gamma { ... };
}

struct TabSample : sqlpp::table_t<TabSample,
                                  TabSample_::Alpha,
                                  TabSample_::Beta,
                                  TabSample_::Gamma> {
    using _value_type = sqlpp::no_value_t;
    struct _alias_t {
        static constexpr const char _literal[] = "tab_sample"; // 表名
        using _name_t = sqlpp::make_char_sequence<sizeof(_literal), _literal>;

        template <typename T>
        struct _member_t {
            T tabSample;
            T& operator()() { return tabSample; }
            const T& operator()() const { return tabSample; }
        };
    };
};
```

### 2.2 魔法在 sqlpp::table_t

```cpp
namespace sqlpp {
// workaround for msvc bug https://connect.microsoft.com/VisualStudio/Feedback/Details/2173198
//  template <typename Table, typename... ColumnSpec>
//  struct table_t : public table_base_t, public member_t<ColumnSpec, column_t<Table, ColumnSpec>>...
template <typename Table, typename... ColumnSpec>
struct table_t : public ColumnSpec::_alias_t::template _member_t<column_t<Table, ColumnSpec>>... {
    using _traits = make_traits<no_value_t, tag::is_raw_table, tag::is_table>;

    using _nodes = detail::type_vector<>;
    using _provided_tables = detail::type_set<Table>;

    static_assert(sizeof...(ColumnSpec) > 0,
                  "at least one column required per table");

    using _required_insert_columns =
        typename detail::make_type_set_if<require_insert_t,
                                          column_t<Table, ColumnSpec>...>::type;
    using _column_tuple_t = std::tuple<column_t<Table, ColumnSpec>...>;

    template <typename AliasProvider, typename T>
    using _foreign_table_alias_t = table_alias_t<AliasProvider, T, ColumnSpec...>;

    template <typename AliasProvider>
    using _alias_t = table_alias_t<AliasProvider, Table, ColumnSpec...>;

    template <typename T>
    auto join(T t) const -> decltype(::sqlpp::join(std::declval<Table>(), t)) {
        return ::sqlpp::join(*static_cast<const Table*>(this), t);
        // *static_cast<const Table*>(this) 只是获取 auto this const& (C++23)
    }

    template <typename T>
    auto inner_join(T t) const; // 以下省略

    template <typename T>
    auto left_outer_join(T t) const;

    template <typename T>
    auto right_outer_join(T t) const;

    template <typename T>
    auto outer_join(T t) const;

    template <typename AliasProvider>
    _alias_t<AliasProvider> as(const AliasProvider& /*unused*/);

    template <typename T>
    auto cross_join(T t) const;
};

template <typename Context, typename Table, typename... ColumnSpec>
Context& serialize(const table_t<Table, ColumnSpec...>& /*unused*/, Context& context) {
    context << name_of<Table>::template char_ptr<Context>();
    return context;
}
} // namespace sqlpp
```

### 2.3 table_t 的父类

```cpp
template <typename Table, typename... ColumnSpec>
struct table_t : public ColumnSpec::_alias_t::template _member_t<column_t<Table, ColumnSpec>>...;
```

其中 `ColumnSpec::_alias_t`, 就例如:

```cpp
struct Alpha { struct _alias_t {
    template <typename T>
    struct _member_t {
        T alpha;
        T& operator()() { return alpha; }
        const T& operator()() const { return alpha; }
    };
};};
```

实际上你就已经明白如何实现 `表名.字段名` 了, 因为它直接继承了成员名称 `T alpha;`, 所以子类可以直接使用.

对于 `T` 是什么, 以及它所如何实现字段之间兼容使用运算符`+`/`>`...的? 我们在看看 `column_t<Table, ColumnSpec>`的实现

### 2.4 column_t

```cpp
namespace sqlpp {
template <typename Table, typename ColumnSpec>
struct column_t : public expression_operators<column_t<Table, ColumnSpec>, value_type_of<ColumnSpec>>,
                  public column_operators<column_t<Table, ColumnSpec>, value_type_of<ColumnSpec>> {
    struct _traits {
        using _value_type = value_type_of<ColumnSpec>;
        using _tags =
            detail::make_joined_set_t<detail::type_set<tag::is_column, tag::is_expression, tag::is_selectable>,
                                      typename ColumnSpec::_traits::_tags>;
    };

    using _nodes = detail::type_vector<>;
    using _required_tables = detail::type_set<Table>;
    using _can_be_null = column_spec_can_be_null_t<ColumnSpec>;

    using _spec_t = ColumnSpec;
    using _table = Table;
    using _alias_t = typename _spec_t::_alias_t;

    template <typename T>
    using _is_valid_assignment_operand = is_valid_assignment_operand<value_type_of<ColumnSpec>, T>;

    // disambiguation for C++20 / clang
    // (see https://bugs.llvm.org/show_bug.cgi?id=46508)
    using expression_operators<column_t<Table, ColumnSpec>, value_type_of<ColumnSpec>>::operator==;
    using expression_operators<column_t<Table, ColumnSpec>, value_type_of<ColumnSpec>>::operator!=;

    column_t() = default;
    column_t(const column_t&) = default;
    column_t(column_t&&) = default;
    column_t& operator=(const column_t&) = default;
    column_t& operator=(column_t&&) = default;
    ~column_t() = default;

    template <typename T = _table>
    auto table() const -> _table {
        static_assert(is_table_t<T>::value, "cannot call get_table for columns of a sub-selects or cte");
        return _table{};
    }

    template <typename alias_provider>
    expression_alias_t<column_t, alias_provider> as(const alias_provider& /*unused*/) const {
        return {*this};
    }

    template <typename T>
    auto operator=(T t) const -> assignment_t<column_t, wrap_operand_t<T>> {
        using rhs = wrap_operand_t<T>;
        static_assert(_is_valid_assignment_operand<rhs>::value, "invalid rhs assignment operand");

        return {*this, {rhs{t}}};
    }

    auto operator=(null_t /*unused*/) const -> assignment_t<column_t, null_t> {
        static_assert(can_be_null_t<column_t>::value, "column cannot be null");
        return {*this, null_t{}};
    }

    auto operator=(default_value_t /*unused*/) const -> assignment_t<column_t, default_value_t> {
        return {*this, default_value_t{}};
    }
};

template <typename Context, typename Table, typename ColumnSpec>
Context& serialize(const column_t<Table, ColumnSpec>&, Context& context) {
    using T = column_t<Table, ColumnSpec>; 
    context << name_of<typename T::_table>::template char_ptr<Context>() << '.'
            << name_of<T>::template char_ptr<Context>();
    return context;
}
} // namespace sqlpp
```

可见, 此处定义了 `operator=` 对于赋值 (`set(tab.name = "xxx")`)、空 (`null_t`)、默认值的情况.

它们都继承自 `struct expression_operators`:

```cpp [expr-主模板]
namespace sqlpp {
template <typename Expr, typename ValueType>
struct expression_operators {
    // wrong_t 始终返回 false, ps: 新版本C++可以直接 !sizeof(Expr)
    static_assert(wrong_t<expression_operators>::value, "Missing expression operators for ValueType");
};
} // namespace sqlpp
```

```cpp [expr-偏特化]
include/sqlpp11/data_types/blob/expression_operators.h:
  55    template <typename Expression>
  56:   struct expression_operators<Expression, blob> : public basic_expression_operators<Expression>
  57    {

include/sqlpp11/data_types/boolean/expression_operators.h:
  36    template <typename Expression>
  37:   struct expression_operators<Expression, boolean> : public basic_expression_operators<Expression>
  38    {

include/sqlpp11/data_types/day_point/expression_operators.h:
  36    template <typename Expression>
  37:   struct expression_operators<Expression, day_point> : public basic_expression_operators<Expression>
  38    {

include/sqlpp11/data_types/floating_point/expression_operators.h:
  38    template <typename Expression>
  39:   struct expression_operators<Expression, floating_point> : public basic_expression_operators<Expression>
  40    {

include/sqlpp11/data_types/integral/expression_operators.h:
  39    template <typename Expression>
  40:   struct expression_operators<Expression, integral> : public basic_expression_operators<Expression>
  41    {

include/sqlpp11/data_types/no_value/expression_operators.h:
  35    template <typename Expression>
  36:   struct expression_operators<Expression, no_value_t> : public basic_expression_operators<Expression>
  37    {

include/sqlpp11/data_types/text/expression_operators.h:
  50    template <typename Expression>
  51:   struct expression_operators<Expression, text> : public basic_expression_operators<Expression>
  52    {

include/sqlpp11/data_types/time_of_day/expression_operators.h:
  37    template <typename Expression>
  38:   struct expression_operators<Expression, time_of_day> : public basic_expression_operators<Expression>
  39    {

include/sqlpp11/data_types/time_point/expression_operators.h:
  37    template <typename Expression>
  38:   struct expression_operators<Expression, time_point> : public basic_expression_operators<Expression>
  39    {

include/sqlpp11/data_types/unsigned_integral/expression_operators.h:
  41    template <typename Expression>
  42:   struct expression_operators<Expression, unsigned_integral> : public basic_expression_operators<Expression>
  43    {
```

```cpp [expr-运算符]
template <typename Expr>
struct basic_expression_operators {
    template <template <typename Lhs, typename Rhs> class NewExpr, typename T>
    struct _new_binary_expression {
        using type = comparison_expression_t<NewExpr, Expr, T>;
    };
    template <template <typename Lhs, typename Rhs> class NewExpr, typename T>
    using _new_binary_expression_t = typename _new_binary_expression<NewExpr, T>::type;

    template <template <typename Lhs, typename... Rhs> class NewExpr, typename... T>
    struct _new_nary_expression {
        using _check = check_in_t<Expr, T...>;
        using type = in_expression_t<_check, NewExpr, Expr, wrap_operand_t<T>...>;
    };

    template <typename T>
    auto operator==(T t) const -> _new_binary_expression_t<equal_to_t, T> {
        using rhs = wrap_operand_t<T>;
        check_comparison_t<Expr, rhs>::verify();
        return {*static_cast<const Expr*>(this), rhs{t}};
    }

    template <typename T>
    auto operator!=(T t) const -> _new_binary_expression_t<not_equal_to_t, T> {
        using rhs = wrap_operand_t<T>;
        check_comparison_t<Expr, rhs>::verify();
        return {*static_cast<const Expr*>(this), rhs{t}};
    }

    template <typename T>
    auto operator<(T t) const -> _new_binary_expression_t<less_than_t, T> {
        using rhs = wrap_operand_t<T>;
        check_comparison_t<Expr, rhs>::verify();
        return {*static_cast<const Expr*>(this), rhs{t}};
    }

    template <typename T>
    auto operator<=(T t) const -> _new_binary_expression_t<less_equal_t, T> {
        using rhs = wrap_operand_t<T>;
        check_comparison_t<Expr, rhs>::verify();
        return {*static_cast<const Expr*>(this), rhs{t}};
    }

    template <typename T>
    auto operator>(T t) const -> _new_binary_expression_t<greater_than_t, T> {
        using rhs = wrap_operand_t<T>;
        check_comparison_t<Expr, rhs>::verify();
        return {*static_cast<const Expr*>(this), rhs{t}};
    }

    template <typename T>
    auto operator>=(T t) const -> _new_binary_expression_t<greater_equal_t, T> {
        using rhs = wrap_operand_t<T>;
        check_comparison_t<Expr, rhs>::verify();
        return {*static_cast<const Expr*>(this), rhs{t}};
    }

    auto is_null() const -> is_null_t<Expr> {
        return {*static_cast<const Expr*>(this)};
    }

    auto is_not_null() const -> is_not_null_t<Expr> {
        return {*static_cast<const Expr*>(this)};
    }

    auto asc() const -> sort_order_t<Expr> {
        return {*static_cast<const Expr*>(this), sort_type::asc};
    }

    auto desc() const -> sort_order_t<Expr> {
        return {*static_cast<const Expr*>(this), sort_type::desc};
    }

    auto order(sort_type s) const -> sort_order_t<Expr> {
        return {*static_cast<const Expr*>(this), s};
    }

    template <typename... T>
    auto in(T... t) const -> typename _new_nary_expression<in_t, T...>::type {
        check_in_t<Expr, wrap_operand_t<T>...>::verify();
        return {*static_cast<const Expr*>(this), typename wrap_operand<T>::type{t}...};
    }

    template <typename... T>
    auto not_in(T... t) const -> typename _new_nary_expression<not_in_t, T...>::type {
        check_in_t<Expr, wrap_operand_t<T>...>::verify();
        return {*static_cast<const Expr*>(this), typename wrap_operand<T>::type{t}...};
    }

    template <typename Defer = void>
    auto operator not() const -> return_type_not_t<Expr, Defer> {
        return_type_not<Expr, Defer>::check::verify();
        return {*static_cast<const Expr*>(this)};
    }

    template <typename R>
    auto operator and(const R& r) const -> return_type_and_t<Expr, R> {
        return_type_and<Expr, R>::check::verify();
        return {*static_cast<const Expr*>(this), wrap_operand_t<R>{r}};
    }

    template <typename R>
    auto operator&(const R& r) const -> return_type_bitwise_and_t<Expr, R> {
        return_type_bitwise_and<Expr, R>::check::verify();
        return {*static_cast<const Expr*>(this), wrap_operand_t<R>{r}};
    }

    template <typename R>
    auto operator|(const R& r) const -> return_type_bitwise_or_t<Expr, R> {
        return_type_bitwise_or<Expr, R>::check::verify();
        return {*static_cast<const Expr*>(this), wrap_operand_t<R>{r}};
    }

    template <typename R>
    auto operator or(const R& r) const -> return_type_or_t<Expr, R> {
        return_type_or<Expr, R>::check::verify();
        return {*static_cast<const Expr*>(this), wrap_operand_t<R>{r}};
    }

    template <typename R>
    auto operator+(const R& r) const -> return_type_plus_t<Expr, R> {
        return_type_plus<Expr, R>::check::verify();
        return {*static_cast<const Expr*>(this), wrap_operand_t<R>{r}};
    }

    template <typename R>
    auto operator-(const R& r) const -> return_type_minus_t<Expr, R> {
        return_type_minus<Expr, R>::check::verify();
        return {*static_cast<const Expr*>(this), wrap_operand_t<R>{r}};
    }

    template <typename R>
    auto operator*(const R& r) const -> return_type_multiplies_t<Expr, R> {
        return_type_multiplies<Expr, R>::check::verify();
        return {*static_cast<const Expr*>(this), wrap_operand_t<R>{r}};
    }

    template <typename R>
    auto operator/(const R& r) const -> return_type_divides_t<Expr, R> {
        return_type_divides<Expr, R>::check::verify();
        return {*static_cast<const Expr*>(this), wrap_operand_t<R>{r}};
    }

    template <typename R>
    auto operator%(const R& r) const -> return_type_modulus_t<Expr, R> {
        return_type_modulus<Expr, R>::check::verify();
        return {*static_cast<const Expr*>(this), wrap_operand_t<R>{r}};
    }

    template <typename Defer = void>
    auto operator+() const -> return_type_unary_plus_t<Expr, Defer> {
        return_type_unary_plus<Expr, Defer>::check::verify();
        return {*static_cast<const Expr*>(this)};
    }

    template <typename Defer = void>
    auto operator-() const -> return_type_unary_minus_t<Expr, Defer> {
        return_type_unary_minus<Expr, Defer>::check::verify();
        return {*static_cast<const Expr*>(this)};
    }

    template <typename R>
    auto operator<<(const R& r) const -> return_type_shift_left_t<Expr, R> {
        return_type_shift_left<Expr, R>::check::verify();
        return {*static_cast<const Expr*>(this), wrap_operand_t<R>{r}};
    }

    template <typename R>
    auto operator>>(const R& r) const -> return_type_shift_right_t<Expr, R> {
        return_type_shift_right<Expr, R>::check::verify();
        return {*static_cast<const Expr*>(this), wrap_operand_t<R>{r}};
    }
};
```

我们先看 `ValueType` 是什么: (`[ValueType = value_type_of<ColumnSpec>]`)

```cpp
struct no_value_t;
namespace detail {
template <typename T, typename Enable = void>
struct value_type_of_impl {
    using type = no_value_t;
};

template <typename T> // 要求 T 存在 _traits::_value_type 类型, 
                      // 比如 Alpha::_traits = sqlpp::make_traits<::sqlpp::bigint, ...>::_value_type;
struct value_type_of_impl<T, detail::void_t<typename T::_traits::_value_type>> {
    using type = typename T::_traits::_value_type;
};
}  // namespace detail

template <typename T>
using value_type_of = typename detail::value_type_of_impl<T>::type;
```

所以就是 `sqlpp::make_traits<::sqlpp::bigint, ...>::_value_type`

所以我们看看 `make_traits` 是什么:

```cpp
template <typename ValueType, typename... Tags>
struct make_traits {
    using _value_type = ValueType; // <--
    using _tags = detail::make_type_set_t<Tags...>;
};
```

原来就是 `make_traits`的第一个模板参数啊~

查阅 `py` 源码, 发现它一定是 sql 类型:

```py
# scripts/ddl2cpp
traitslist = ["sqlpp::" + columnType] # 初次定义
# ...
print(
    "      using _traits = sqlpp::make_traits<"
    + ", ".join(traitslist)
    + ">;",
    file=header,
)
```

因此, `ValueType` 是字段SQL类型, 然后通过 `expression_operators`偏特化继承到对应的`运算符`, `column_operators`是继承`+=`/`-=` 这类运算符, 暂时没有看到何处使用.

> [!TIP]
> 你可能有疑问: `struct column_t : expression_operators<column_t<Table, ColumnSpec>, value_type_of<ColumnSpec>>` 不是递归了 `column_t<Table, ColumnSpec>` 了吗?
>
>   - 实际上这类似于 `CRTP`, 模板参数允许使用不完整类型 (毕竟可以二阶段名称查找)

综上, 即可做到基于不同的 SQL类型, 提供不同的运算符, 并且在编译期诊断. 而且可以直接使用名称. (注: 仅说明了初步的定义架构, 内部的执行逻辑并未涉及~)

总之还是非常精密的! 从晚上12点看到1点了, 先睡觉了~