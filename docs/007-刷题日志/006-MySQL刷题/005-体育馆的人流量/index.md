# 601. 体育馆的人流量
原题: [601. 体育馆的人流量](https://leetcode.cn/problems/human-traffic-of-stadium/description/)

困难


```sql
表：Stadium
+---------------+---------+
| Column Name   | Type    |
+---------------+---------+
| id            | int     |
| visit_date    | date    |
| people        | int     |
+---------------+---------+
visit_date 是该表中具有唯一值的列。
每日人流量信息被记录在这三列信息中：序号 (id)、日期 (visit_date)、 人流量 (people)
每天只有一行记录，日期随着 id 的增加而增加
 

编写解决方案找出每行的人数大于或等于 100 且 id 连续的三行或更多行记录。

返回按 visit_date 升序排列 的结果表。

查询结果格式如下所示。

 

示例 1:

输入：
Stadium 表:
+------+------------+-----------+
| id   | visit_date | people    |
+------+------------+-----------+
| 1    | 2017-01-01 | 10        |
| 2    | 2017-01-02 | 109       |
| 3    | 2017-01-03 | 150       |
| 4    | 2017-01-04 | 99        |
| 5    | 2017-01-05 | 145       |
| 6    | 2017-01-06 | 1455      |
| 7    | 2017-01-07 | 199       |
| 8    | 2017-01-09 | 188       |
+------+------------+-----------+
输出：
+------+------------+-----------+
| id   | visit_date | people    |
+------+------------+-----------+
| 5    | 2017-01-05 | 145       |
| 6    | 2017-01-06 | 1455      |
| 7    | 2017-01-07 | 199       |
| 8    | 2017-01-09 | 188       |
+------+------------+-----------+
解释：
id 为 5、6、7、8 的四行 id 连续，并且每行都有 >= 100 的人数记录。
请注意，即使第 7 行和第 8 行的 visit_date 不是连续的，输出也应当包含第 8 行，因为我们只需要考虑 id 连续的记录。
不输出 id 为 2 和 3 的行，因为至少需要三条 id 连续的记录。
```

# 题解
## 我的八嘎代码

直接暴力 列转行(满足人数 且 id - 1 == id(主) == id + 1), 行又转列(~~一行mysql呀~直接开除!~~)

- 学到了: [一篇文章解析mysql的 行转列（7种方法） 和 列转行](https://blog.csdn.net/weter_drop/article/details/105899362)

```sql
SELECT 
    aa.aid as id,
    aa.avisit_date AS visit_date,
    aa.apeople AS people 
FROM 
    (
    select
        b.id as aid,
        b.visit_date as avisit_date,
        b.people as apeople,
        c.id as bid,
        c.visit_date as bvisit_date,
        c.people as bpeople,
        d.id as cid,
        d.visit_date as cvisit_date,
        d.people as cpeople
    from
        (
        select
            a.*
        from
            Stadium as a
        where
            a.people >= 100
        ) as b
    join
        (
        select
            a.*
        from
            Stadium as a
        where
            a.people >= 100
        ) as c
    on
        b.id + 1 = c.id
    join
        (
        select
            a.*
        from
            Stadium as a
        where
            a.people >= 100
        ) as d
    on
        c.id + 1 = d.id
) as aa
UNION 
SELECT
    bb.bid as id,
    bb.bvisit_date AS visit_date,
    bb.bpeople AS people 
FROM 
    (
    select
        b.id as aid,
        b.visit_date as avisit_date,
        b.people as apeople,
        c.id as bid,
        c.visit_date as bvisit_date,
        c.people as bpeople,
        d.id as cid,
        d.visit_date as cvisit_date,
        d.people as cpeople
    from
        (
        select
            a.*
        from
            Stadium as a
        where
            a.people >= 100
        ) as b
    join
        (
        select
            a.*
        from
            Stadium as a
        where
            a.people >= 100
        ) as c
    on
        b.id + 1 = c.id
    join
        (
        select
            a.*
        from
            Stadium as a
        where
            a.people >= 100
        ) as d
    on
        c.id + 1 = d.id
    ) as bb
UNION 
SELECT
    cc.cid as id,
    cc.cvisit_date AS visit_date,
    cc.cpeople AS people 
FROM 
    (
    select
        b.id as aid,
        b.visit_date as avisit_date,
        b.people as apeople,
        c.id as bid,
        c.visit_date as bvisit_date,
        c.people as bpeople,
        d.id as cid,
        d.visit_date as cvisit_date,
        d.people as cpeople
    from
        (
        select
            a.*
        from
            Stadium as a
        where
            a.people >= 100
        ) as b
    join
        (
        select
            a.*
        from
            Stadium as a
        where
            a.people >= 100
        ) as c
    on
        b.id + 1 = c.id
    join
        (
        select
            a.*
        from
            Stadium as a
        where
            a.people >= 100
        ) as d
    on
        c.id + 1 = d.id
) as cc
ORDER BY id;
```

<div style="margin-top: 80px;">

---
</div>

## 正解

链接 [图解——连续日期及难点分析](https://leetcode.cn/problems/human-traffic-of-stadium/solutions/701681/tu-jie-lian-xu-ri-qi-ji-nan-dian-fen-xi-xnj58/)

在公司用`hive`常会用到这解法，有时候会在临时表里再多重嵌套

这道题需要提前用`With`临时空间，是因为`where`子句中需要再次调用`from`中选取的表

这里再聊一下sql的运行顺序：

- from -> where -> group by -> select -> order by -> limit

即临时表`t1`需要再`from`和`where`中都用到因此需要提前定义

天才!:
```sql
with t1 as(
    select
        *,
        id - row_number() over(order by id) as rk -- 先满足people >= 100再标上序号
    from stadium
    where people >= 100
)

select id,visit_date,people
from t1
where rk in(
    select rk
    from t1
    group by rk -- 按照序号分组, (为什么可以分组, 因为相差相同(见下面))
    having count(rk) >= 3 -- 大于三个
)

-- 作者：宇航员
-- 链接：https://leetcode.cn/problems/human-traffic-of-stadium/solutions/701681/tu-jie-lian-xu-ri-qi-ji-nan-dian-fen-xi-xnj58/
-- 来源：力扣（LeetCode）
-- 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```

示例

输入:

```sql
| id | visit_date | people |
| -- | ---------- | ------ |
| 1  | 2017-01-01 | 10     |
| 2  | 2017-01-02 | 109    |
| 3  | 2017-01-03 | 150    |
| 4  | 2017-01-04 | 999    |
| 5  | 2017-01-05 | 0      |
| 6  | 2017-01-06 | 1455   |
| 7  | 2017-01-07 | 199    |
| 8  | 2017-01-09 | 188    |
| 9  | 2024-03-14 | 100    |
```

代码:
```sql
select 
    *, 
    id as `|`, 
    row_number() over(order by id) as c ,
    id - row_number() over(order by id) as rk
from stadium
where people >= 100
```

得:
```sql
| id | visit_date | people | | | c | rk |
| -- | ---------- | ------ | - | - | -- |
| 2  | 2017-01-02 | 109    | 2 | 1 | 1  |
| 3  | 2017-01-03 | 150    | 3 | 2 | 1  |
| 4  | 2017-01-04 | 999    | 4 | 3 | 1  |
| 6  | 2017-01-06 | 1455   | 6 | 4 | 2  |
| 7  | 2017-01-07 | 199    | 7 | 5 | 2  |
| 8  | 2017-01-09 | 188    | 8 | 6 | 2  |
| 9  | 2024-03-14 | 100    | 9 | 7 | 2  |
```