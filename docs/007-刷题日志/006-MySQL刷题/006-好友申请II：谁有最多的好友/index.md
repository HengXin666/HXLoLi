# 602. 好友申请 II ：谁有最多的好友

题目: [602. 好友申请 II ：谁有最多的好友](https://leetcode.cn/problems/friend-requests-ii-who-has-the-most-friends/description/)

```sql
RequestAccepted 表：

+----------------+---------+
| Column Name    | Type    |
+----------------+---------+
| requester_id   | int     |
| accepter_id    | int     |
| accept_date    | date    |
+----------------+---------+
(requester_id, accepter_id) 是这张表的主键(具有唯一值的列的组合)。
这张表包含发送好友请求的人的 ID ，接收好友请求的人的 ID ，以及好友请求通过的日期。
 

编写解决方案，找出拥有最多的好友的人和他拥有的好友数目。

生成的测试用例保证拥有最多好友数目的只有 1 个人。

查询结果格式如下例所示。

 

示例 1：

输入：
RequestAccepted 表：
+--------------+-------------+-------------+
| requester_id | accepter_id | accept_date |
+--------------+-------------+-------------+
| 1            | 2           | 2016/06/03  |
| 1            | 3           | 2016/06/08  |
| 2            | 3           | 2016/06/08  |
| 3            | 4           | 2016/06/09  |
+--------------+-------------+-------------+
输出：
+----+-----+
| id | num |
+----+-----+
| 3  | 3   |
+----+-----+
解释：
编号为 3 的人是编号为 1 ，2 和 4 的人的好友，所以他总共有 3 个好友，比其他人都多。
 

进阶：在真实世界里，可能会有多个人拥有好友数相同且最多，你能找到所有这些人吗？
```

# 题解
## 我代码
暴力了, 好友 = 我加你 + 他加我, 但是有两列, 有的id有的没有, 所以`on`会出现null, 只好左链接 和 右链接, 然后取最大值

实现的时候, 就是合并

```sql
SELECT id, MAX(num) AS num
FROM (
    SELECT a.id, IF(a.num IS NULL, 0, a.num) + IF(b.num IS NULL, 0, b.num) AS num
    FROM (
        SELECT accepter_id AS id, COUNT(accepter_id) AS num
        FROM RequestAccepted
        GROUP BY accepter_id
    ) AS a
    LEFT JOIN (
        SELECT requester_id AS id, COUNT(requester_id) AS num
        FROM RequestAccepted
        GROUP BY requester_id
    ) AS b ON a.id = b.id
    
    UNION ALL
    
    SELECT b.id, IF(a.num IS NULL, 0, a.num) + IF(b.num IS NULL, 0, b.num) AS num
    FROM (
        SELECT accepter_id AS id, COUNT(accepter_id) AS num
        FROM RequestAccepted
        GROUP BY accepter_id
    ) AS a
    RIGHT JOIN (
        SELECT requester_id AS id, COUNT(requester_id) AS num
        FROM RequestAccepted
        GROUP BY requester_id
    ) AS b ON a.id = b.id
) AS combined_table
GROUP BY id
ORDER BY num desc
limit 0, 1;
```

## 正解

还不如直接把 `我加你` 和 `他加我` 合并, 然后在取值

好友关系是相互的。A加过B好友后，A和B相互是好友了。

那么，将表中的字段 $requester\_id$ 和 $accepter\_id$ 交换后，再拼接起来。能找出全部的好友关系。

应用UNION运算符

```sql
SELECT column_list
UNION [DISTINCT | ALL] -- 即使不用DISTINCT关键字，UNION也会删除重复行。ALL不会删除重复行
SELECT column_list
```

按rid分组，计算每组的好友个数，并按好友个数降序，取第一个人。

```sql
select rid as `id`,count(aid) as `num`
from
(
    select
        R1.requester_id as rid,
        R1.accepter_id as aid
    from
        request_accepted as R1
    UNION all
    select
        R2.accepter_id as rid,
        R2.requester_id as aid
    from
        request_accepted as R2
) as A
group by rid
order by num desc
limit 0,1

-- 作者：jason
-- 链接：https://leetcode.cn/problems/friend-requests-ii-who-has-the-most-friends/solutions/20727/bu-yong-qu-zhong-ke-de-zheng-que-jie-guo-by-jason-/
-- 来源：力扣（LeetCode）
-- 著作权归作者所有。商业转载请联系作者获得授权，非商业转载请注明出处。
```
